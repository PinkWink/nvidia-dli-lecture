"""강의 32: PPO 설정 (rsl_rl)."""
from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class PendulumPpoCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 32                # 정책을 한 번 고치기 전에 모으는 스텝 수 (0.55초)
    max_iterations = 300
    save_interval = 100
    experiment_name = "pinklab_pendulum"
    obs_groups = {"policy": ["policy"], "critic": ["policy"]}
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_obs_normalization=True,     # 관측의 크기가 제각각이라 (sin·cos 는 1, 속도는 20) 맞춰 준다
        critic_obs_normalization=True,
        actor_hidden_dims=[64, 64],
        critic_hidden_dims=[128, 128],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.004,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.995,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
