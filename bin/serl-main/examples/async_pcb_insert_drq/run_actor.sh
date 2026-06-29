export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.1 && \
DEMO_PATH="${DEMO_PATH:-pcb_insert_demos.pkl}" && \
CHECKPOINT_PATH="${CHECKPOINT_PATH:-checkpoints/pcb_insert_drq}" && \
EVAL_CHECKPOINT_STEP="${EVAL_CHECKPOINT_STEP:-0}" && \
EVAL_N_TRAJS="${EVAL_N_TRAJS:-5}" && \
python async_drq_randomized.py "$@" \
    --actor \
    --render \
    --env FrankaPCBInsert-Vision-v0 \
    --exp_name=serl_dev_drq_rlpd_pcb_insert_random_resnet \
    --seed 0 \
    --random_steps 0 \
    --encoder_type resnet-pretrained \
    --demo_path "$DEMO_PATH" \
    --checkpoint_path "$CHECKPOINT_PATH" \
    --eval_checkpoint_step "$EVAL_CHECKPOINT_STEP" \
    --eval_n_trajs "$EVAL_N_TRAJS"
