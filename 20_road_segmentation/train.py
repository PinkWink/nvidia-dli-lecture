"""강의 20: 도로를 알아보는 세그먼테이션 모델을 학습합니다 (Isaac Sim 에 들어 있는 PyTorch 사용).

    ~/isaacsim/python.sh 20_road_segmentation/train.py                  # 학습 (work/road_model 에 저장)
    ~/isaacsim/python.sh 20_road_segmentation/train.py --epochs 60      # 더 오래
    ~/isaacsim/python.sh 20_road_segmentation/train.py --data work/road_dataset --out work/road_model

필요한 것: 강의 19 의 데이터 (work/road_dataset). 시뮬레이터 창은 뜨지 않습니다 — python.sh 를 파이썬 실행기로만 씁니다.

만드는 것 (work/road_model)
    road_seg.pt         가장 잘 맞힌 시점의 모델 (PyTorch)
    road_seg.onnx       같은 모델을 다른 프로그램에서 쓰는 형식 (다음 강의의 ROS 2 노드가 읽는다)
    history.csv · training.png      에포크마다의 손실 · 정확도
    predictions.png     검증 영상 6장: 영상 | 정답 | 모델의 답
"""
import argparse
import csv
import os
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
NAMES = ["그 밖", "보도 · 공원", "도로", "중앙선", "횡단보도 · 정지선"]            # 종류 번호 0 ~ 4 (강의 19)
PALETTE = np.array([(40, 40, 48), (232, 150, 190), (90, 96, 110), (250, 204, 60), (255, 255, 255)], dtype=np.uint8)
WIDTH, HEIGHT = 320, 180              # 모델에 넣는 크기 (원본 640 × 360 의 절반). 작을수록 빠르다
MEAN, STD = 0.5, 0.25                 # 입력 정규화: (픽셀 / 255 − MEAN) / STD


# ---------------------------------------------------------------- 데이터
def load_split(data, name):
    """목록 파일(train.txt · val.txt)의 영상과 정답을 한꺼번에 읽어 텐서로 돌려준다 (400장이면 메모리에 다 들어간다)."""
    with open(os.path.join(data, name + ".txt")) as f:
        ids = f.read().split()
    images = np.stack([np.asarray(Image.open(os.path.join(data, "images", i + ".png")).convert("RGB").resize((WIDTH, HEIGHT), Image.BILINEAR)) for i in ids])
    masks = np.stack([np.asarray(Image.open(os.path.join(data, "masks", i + ".png")).resize((WIDTH, HEIGHT), Image.NEAREST)) for i in ids])
    return torch.from_numpy(images).permute(0, 3, 1, 2).contiguous(), torch.from_numpy(masks).long()       # (N, 3, H, W) uint8, (N, H, W)


def normalize(images):
    return (images.float() / 255.0 - MEAN) / STD


def augment(images, masks):
    """학습할 때마다 영상을 조금씩 바꾼다: 좌우 뒤집기, 밝기 · 대비 · 색 바꾸기."""
    flip = torch.rand(images.shape[0], device=images.device) < 0.5
    images = torch.where(flip[:, None, None, None], images.flip(-1), images)
    masks = torch.where(flip[:, None, None], masks.flip(-1), masks)
    x = images.float() / 255.0
    n = x.shape[0]
    x = x * torch.empty(n, 1, 1, 1, device=x.device).uniform_(0.7, 1.3)                    # 밝기
    mean = x.mean(dim=(1, 2, 3), keepdim=True)
    x = (x - mean) * torch.empty(n, 1, 1, 1, device=x.device).uniform_(0.75, 1.25) + mean  # 대비
    x = x * torch.empty(n, 3, 1, 1, device=x.device).uniform_(0.9, 1.1)                    # 색 (채널별)
    x = x + torch.randn_like(x) * 0.02                                                      # 잡음
    return (x.clamp(0.0, 1.0) - MEAN) / STD, masks


# ---------------------------------------------------------------- 모델: 작은 U-Net
def block(cin, cout):
    return nn.Sequential(nn.Conv2d(cin, cout, 3, padding=1, bias=False), nn.BatchNorm2d(cout), nn.ReLU(inplace=True),
                         nn.Conv2d(cout, cout, 3, padding=1, bias=False), nn.BatchNorm2d(cout), nn.ReLU(inplace=True))


class RoadNet(nn.Module):
    """영상을 줄여 가며 특징을 뽑고(encoder), 다시 키워 가며 픽셀마다 종류를 정한다(decoder). 줄이기 전의 특징을 건너 붙인다."""

    def __init__(self, classes=len(NAMES), width=16):
        super().__init__()
        w = width
        self.enc1, self.enc2, self.enc3, self.enc4 = block(3, w), block(w, w * 2), block(w * 2, w * 4), block(w * 4, w * 8)
        self.dec3, self.dec2, self.dec1 = block(w * 8 + w * 4, w * 4), block(w * 4 + w * 2, w * 2), block(w * 2 + w, w)
        self.head = nn.Conv2d(w, classes, 1)

    def forward(self, x):
        e1 = self.enc1(x)                                    # 1/1 크기
        e2 = self.enc2(F.max_pool2d(e1, 2))                  # 1/2
        e3 = self.enc3(F.max_pool2d(e2, 2))                  # 1/4
        e4 = self.enc4(F.max_pool2d(e3, 2))                  # 1/8
        d3 = self.dec3(torch.cat([F.interpolate(e4, size=e3.shape[-2:], mode="bilinear", align_corners=False), e3], dim=1))
        d2 = self.dec2(torch.cat([F.interpolate(d3, size=e2.shape[-2:], mode="bilinear", align_corners=False), e2], dim=1))
        d1 = self.dec1(torch.cat([F.interpolate(d2, size=e1.shape[-2:], mode="bilinear", align_corners=False), e1], dim=1))
        return self.head(d1)                                 # (N, 종류 수, H, W): 픽셀마다 종류별 점수


# ---------------------------------------------------------------- 평가
def evaluate(model, images, masks, weight, batch=40):
    """검증 데이터의 손실과 종류별 IoU. IoU = (정답과 모델의 답이 겹치는 넓이) ÷ (둘을 합친 넓이)."""
    model.eval()
    inter, union, loss_sum = torch.zeros(len(NAMES)), torch.zeros(len(NAMES)), 0.0
    predictions = []
    with torch.no_grad():
        for i in range(0, len(images), batch):
            x, y = normalize(images[i:i + batch]), masks[i:i + batch]
            out = model(x)
            loss_sum += F.cross_entropy(out, y, weight=weight).item() * len(x)
            p = out.argmax(1)
            predictions.append(p.cpu())
            for c in range(len(NAMES)):
                inter[c] += ((p == c) & (y == c)).sum().item()
                union[c] += ((p == c) | (y == c)).sum().item()
    return loss_sum / len(images), (inter / union.clamp(min=1)).numpy(), torch.cat(predictions)


def save_predictions(path, images, masks, predictions, count=6):
    rows = []
    for i in np.linspace(0, len(images) - 1, count).astype(int):
        image = images[i].permute(1, 2, 0).cpu().numpy()
        rows.append(np.concatenate([image, PALETTE[masks[i].cpu().numpy()], PALETTE[predictions[i].numpy()]], axis=1))
    Image.fromarray(np.concatenate(rows, axis=0)).save(path)


def save_curves(path, history):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    epochs = [h["epoch"] for h in history]
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4))
    a.plot(epochs, [h["train_loss"] for h in history], label="train")
    a.plot(epochs, [h["val_loss"] for h in history], label="val")
    a.set_xlabel("epoch"); a.set_ylabel("loss"); a.set_yscale("log"); a.legend(); a.grid(alpha=0.3)
    labels = ["other", "sidewalk", "road", "lane line", "crosswalk"]
    for c, label in enumerate(labels):
        b.plot(epochs, [h[f"iou_{c}"] for h in history], label=label)
    b.plot(epochs, [h["miou"] for h in history], "k--", label="mean")
    b.set_xlabel("epoch"); b.set_ylabel("IoU (val)"); b.set_ylim(0, 1); b.legend(loc="lower right"); b.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default=os.path.join(REPO, "work", "road_dataset"), help="강의 19 의 데이터 폴더")
    parser.add_argument("--out", default=os.path.join(REPO, "work", "road_model"), help="모델을 저장할 폴더")
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--lr", type=float, default=2e-3, help="학습률")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    torch.manual_seed(args.seed)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    out = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(out, exist_ok=True)
    if not os.path.exists(os.path.join(args.data, "train.txt")):
        raise SystemExit(f"데이터가 없습니다: {args.data}\n먼저 실행:  ~/isaacsim/python.sh 19_road_dataset/collect_dataset.py")

    train_x, train_y = [t.to(device) for t in load_split(args.data, "train")]
    val_x, val_y = [t.to(device) for t in load_split(args.data, "val")]
    # 픽셀이 적은 종류(중앙선 · 횡단보도)를 놓치지 않도록, 적을수록 큰 가중치를 준다
    share = torch.bincount(train_y.flatten(), minlength=len(NAMES)).float() / train_y.numel()
    weight = (1.0 / share.clamp(min=1e-4)).sqrt()
    weight = (weight / weight.mean()).to(device)
    model = RoadNet().to(device)
    parameters = sum(p.numel() for p in model.parameters())
    print(f"장치 {device} ({torch.cuda.get_device_name(0) if device == 'cuda' else 'CPU'}), PyTorch {torch.__version__}", flush=True)
    print(f"학습 {len(train_x)}장 · 검증 {len(val_x)}장, 입력 {WIDTH} × {HEIGHT}, 모델 파라미터 {parameters:,}개", flush=True)
    print("종류별 픽셀 비율 → 가중치: " + ", ".join(f"{n} {s * 100:.1f}% → {w:.2f}" for n, s, w in zip(NAMES, share.tolist(), weight.tolist())), flush=True)

    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    steps = args.epochs * -(-len(train_x) // args.batch)                 # 전체 학습 걸음 수
    scheduler = torch.optim.lr_scheduler.OneCycleLR(optimizer, max_lr=args.lr, total_steps=steps)
    history, best, start = [], -1.0, time.time()
    print(f"{'에포크':>5} {'학습 손실':>9} {'검증 손실':>9} {'평균 IoU':>8}   " + "  ".join(f"{n}" for n in NAMES), flush=True)
    for epoch in range(1, args.epochs + 1):
        model.train()
        order, total = torch.randperm(len(train_x), device=device), 0.0
        for i in range(0, len(order), args.batch):
            index = order[i:i + args.batch]
            x, y = augment(train_x[index], train_y[index])
            loss = F.cross_entropy(model(x), y, weight=weight)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            scheduler.step()
            total += loss.item() * len(index)
        val_loss, iou, predictions = evaluate(model, val_x, val_y, weight)
        row = {"epoch": epoch, "train_loss": total / len(train_x), "val_loss": val_loss, "miou": float(iou.mean())}
        row.update({f"iou_{c}": float(v) for c, v in enumerate(iou)})
        history.append(row)
        mark = ""
        if row["miou"] > best:
            best, mark = row["miou"], " ← 저장"
            torch.save({"model": model.state_dict(), "width": WIDTH, "height": HEIGHT, "mean": MEAN, "std": STD, "names": NAMES}, os.path.join(out, "road_seg.pt"))
            save_predictions(os.path.join(out, "predictions.png"), val_x, val_y, predictions)
        if epoch == 1 or epoch % 5 == 0 or epoch == args.epochs or mark:
            print(f"{epoch:5d} {row['train_loss']:9.4f} {val_loss:9.4f} {row['miou']:8.3f}   " + "  ".join(f"{v:.3f}" for v in iou) + mark, flush=True)
    print(f"학습 시간 {time.time() - start:.0f}초. 가장 좋은 평균 IoU {best:.3f}", flush=True)

    with open(os.path.join(out, "history.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(history[0]))
        writer.writeheader()
        writer.writerows(history)
    save_curves(os.path.join(out, "training.png"), history)

    # 가장 좋았던 모델을 다시 읽어 ONNX 로 내보낸다: 입력 (1, 3, 180, 320) 정규화된 RGB → 출력 (1, 5, 180, 320) 종류별 점수
    model.load_state_dict(torch.load(os.path.join(out, "road_seg.pt"), map_location=device)["model"])
    model.eval()
    onnx_path = os.path.join(out, "road_seg.onnx")
    torch.onnx.export(model, torch.zeros(1, 3, HEIGHT, WIDTH, device=device), onnx_path, input_names=["image"], output_names=["scores"], opset_version=13)
    _, iou, _ = evaluate(model, val_x, val_y, weight)
    print("저장한 모델의 종류별 IoU (검증):", flush=True)
    for name, v in zip(NAMES, iou):
        print(f"  {name:<12} {v:.3f}", flush=True)
    print(f"저장: {out}/road_seg.pt, road_seg.onnx ({os.path.getsize(onnx_path) / 1e6:.1f} MB), training.png, predictions.png", flush=True)
