"""강의 54: 미리 학습된 AI 모델 넷을 같은 모양으로 부르는 얇은 껍데기. 모두 Hugging Face transformers 로 돌립니다.

    이름        모델                                              넣는 것 → 나오는 것
    Ask         Qwen/Qwen3-VL-2B-Instruct                         영상 + 질문 → 글로 된 답
    Segment     shi-labs/oneformer_ade20k_swin_tiny               영상 → 픽셀마다 무엇인지
    Distance    depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf    영상 한 장 → 픽셀마다 거리 [m]
    Find        google/owlv2-base-patch16-ensemble                영상 + 찾을 것의 이름 → 상자

처음 부를 때 모델을 내려받는다 (~/.cache/huggingface, 넷을 합쳐 5 GB 쯤). 다음부터는 받아 둔 것을 쓴다.
영상은 (높이, 너비, 3) 의 RGB 배열로 넣는다.
"""
import time

import numpy as np
import torch
from PIL import Image

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
try:                                                            # 모델을 읽을 때 나오는 긴 보고는 숨긴다 (오류는 그대로 나온다)
    from huggingface_hub.utils import disable_progress_bars
    from transformers.utils import logging
    logging.set_verbosity_error()
    logging.disable_progress_bar()
    disable_progress_bars()
except ImportError:
    pass


class Model:
    """공통: 모델을 올리는 데 걸린 시간(loaded)과 마지막 한 번에 걸린 시간(took) [s] 을 기억한다."""

    def __init__(self):
        started = time.monotonic()
        self.load()
        self.loaded, self.took = time.monotonic() - started, 0.0

    def __call__(self, *args, **kwargs):
        started = time.monotonic()
        with torch.no_grad():
            answer = self.run(*args, **kwargs)
        self.took = time.monotonic() - started
        return answer


class Ask(Model):
    """말로 묻기: ask(image, "How many cups are there?") → 답 (글)."""
    name = "Qwen/Qwen3-VL-2B-Instruct"

    def load(self):
        from transformers import AutoModelForImageTextToText, AutoProcessor
        self.processor = AutoProcessor.from_pretrained(self.name)
        self.model = AutoModelForImageTextToText.from_pretrained(self.name, dtype=torch.bfloat16 if DEVICE == "cuda" else torch.float32, device_map=DEVICE).eval()

    def run(self, image, question, words=64):
        chat = [{"role": "user", "content": [{"type": "image", "image": Image.fromarray(image)}, {"type": "text", "text": question}]}]
        inputs = self.processor.apply_chat_template(chat, tokenize=True, add_generation_prompt=True, return_dict=True, return_tensors="pt").to(DEVICE)
        output = self.model.generate(**inputs, max_new_tokens=words, do_sample=False)
        return self.processor.decode(output[0][inputs["input_ids"].shape[1]:], skip_special_tokens=True).strip()


class Segment(Model):
    """픽셀마다 무엇인지: segment(image) → [(이름, 화면에서 차지하는 비율)] 큰 것부터. 마스크는 self.mask 에 남는다."""
    name = "shi-labs/oneformer_ade20k_swin_tiny"

    def load(self):
        from transformers import OneFormerForUniversalSegmentation, OneFormerProcessor
        self.processor = OneFormerProcessor.from_pretrained(self.name)
        self.model = OneFormerForUniversalSegmentation.from_pretrained(self.name).to(DEVICE).eval()

    def run(self, image):
        inputs = self.processor(images=Image.fromarray(image), task_inputs=["semantic"], return_tensors="pt").to(DEVICE)
        self.mask = self.processor.post_process_semantic_segmentation(self.model(**inputs), target_sizes=[image.shape[:2]])[0].cpu().numpy()
        kinds, counts = np.unique(self.mask, return_counts=True)
        return sorted(((self.model.config.id2label[int(kind)], count / self.mask.size) for kind, count in zip(kinds, counts)), key=lambda item: -item[1])


class Distance(Model):
    """영상 한 장으로 거리 어림하기: distance(image) → (높이, 너비) 배열 [m]. 깊이 센서 없이 색 영상만 쓴다."""
    name = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"

    def load(self):
        from transformers import pipeline
        self.pipe = pipeline("depth-estimation", model=self.name, device=0 if DEVICE == "cuda" else -1)

    def run(self, image):
        depth = self.pipe(Image.fromarray(image))["predicted_depth"]
        return torch.nn.functional.interpolate(depth[None, None].float(), size=image.shape[:2], mode="bilinear")[0, 0].cpu().numpy()


class Find(Model):
    """이름으로 찾기: find(image, ["a pink cup", "a person"]) → [(이름, 점수, (x1, y1, x2, y2))] 점수가 높은 것부터.

    YOLO 처럼 미리 정한 종류가 아니라 글로 적은 아무 이름이나 찾는다 (학습이 필요 없다).
    """
    name = "google/owlv2-base-patch16-ensemble"

    def load(self):
        from transformers import Owlv2ForObjectDetection, Owlv2Processor
        self.processor = Owlv2Processor.from_pretrained(self.name)
        self.model = Owlv2ForObjectDetection.from_pretrained(self.name).to(DEVICE).eval()

    def run(self, image, names, threshold=0.15):
        height, width = image.shape[:2]
        inputs = self.processor(text=[list(names)], images=Image.fromarray(image), return_tensors="pt").to(DEVICE)
        side = max(height, width)                                      # 이 모델은 영상을 정사각형으로 채워서 본다. 상자도 그 크기로 돌려받는다
        found = self.processor.image_processor.post_process_object_detection(self.model(**inputs), threshold=threshold, target_sizes=torch.tensor([[side, side]]))[0]
        rows = []
        for box, score, label in zip(found["boxes"].cpu().numpy(), found["scores"].cpu().numpy(), found["labels"].cpu().numpy()):
            x1, y1, x2, y2 = box
            rows.append((names[int(label)], float(score), (float(np.clip(x1, 0, width)), float(np.clip(y1, 0, height)), float(np.clip(x2, 0, width)), float(np.clip(y2, 0, height)))))
        return sorted(rows, key=lambda row: -row[1])
