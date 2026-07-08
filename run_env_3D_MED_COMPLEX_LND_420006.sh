
BASE_INDICES="0,1,2,3,4,5"

# Parse flags
USE_CAMERAS=false
for arg in "$@"; do
    case $arg in
        --cameras)
            USE_CAMERAS=true
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [--cameras]"
            exit 1
            ;;
    esac
done

# Build the index list
if [ "$USE_CAMERAS" = true ]; then
    INDICES="${BASE_INDICES},14"
    echo "Launching with generated cameras (camera_generator.yaml)..."
else
    INDICES="${BASE_INDICES}"
    echo "Launching without generated cameras..."
fi

ambf_simulator --launch_file launch.yaml -l ${INDICES} -p 200 -t 1 --override_max_comm_freq 100 --override_min_comm_freq 100
