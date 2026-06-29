export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.2 && \
CHECKPOINT_PATH="${CHECKPOINT_PATH:-checkpoints/pcb_insert_bc}" && \
DEMO_PATHS="${DEMO_PATHS:-bc_demos/pcb_insert_demos.pkl}" && \
EVAL_CHECKPOINT_STEP="${EVAL_CHECKPOINT_STEP:-0}" && \
EVAL_N_TRAJS="${EVAL_N_TRAJS:-5}" && \
DEMO_FLAGS="" && \
for demo_path in $DEMO_PATHS; do DEMO_FLAGS="$DEMO_FLAGS --demo_paths $demo_path"; done && \
python ../bc_policy.py "$@" $DEMO_FLAGS \
    --env FrankaPCBInsert-Vision-v0 \
    --exp_name=serl_dev_bc_pcb_insert_random_resnet \
    --seed 0 \
    --batch_size 256 \
    --max_steps 20000 \
    --remove_xy True \
    --encoder_type resnet-pretrained \
    --checkpoint_path "$CHECKPOINT_PATH" \
    --eval_checkpoint_step "$EVAL_CHECKPOINT_STEP" \
    --eval_n_trajs "$EVAL_N_TRAJS"
