export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.2 && \
DEMO_PATH="${DEMO_PATH:-pcb_insert_demos.pkl}" && \
CHECKPOINT_PATH="${CHECKPOINT_PATH:-checkpoints/pcb_insert_drq}" && \
python async_drq_randomized.py "$@" \
    --learner \
    --env FrankaPCBInsert-Vision-v0 \
    --exp_name=serl_dev_drq_rlpd_pcb_insert_random_resnet \
    --seed 0 \
    --random_steps 1000 \
    --training_starts 200 \
    --critic_actor_ratio 4 \
    --batch_size 256 \
    --eval_period 2000 \
    --encoder_type resnet-pretrained \
    --demo_path "$DEMO_PATH" \
    --checkpoint_period 1000 \
    --checkpoint_path "$CHECKPOINT_PATH"
