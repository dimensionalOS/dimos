
MODULE=""
HARDWARE=0
VERBOSE=0
COVERAGE=0

while [[ $# -gt 0 ]]; do
  case $1 in
    -m|--module)
      MODULE="$2"
      shift 2
      ;;
    --hardware)
      HARDWARE=1
      shift
      ;;
    -v|--verbose)
      VERBOSE=1
      shift
      ;;
    --coverage)
      COVERAGE=1
      shift
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--module MODULE] [--hardware] [--verbose] [--coverage]"
      exit 1
      ;;
  esac
done

CMD="python -m pytest"

if [ $VERBOSE -eq 1 ]; then
  CMD="$CMD -v"
fi

if [ $HARDWARE -eq 0 ]; then
  CMD="$CMD -m 'not hardware'"
fi

if [ $COVERAGE -eq 1 ]; then
  CMD="$CMD --cov=dimos --cov-report=term --cov-report=html"
fi

if [ ! -z "$MODULE" ]; then
  CMD="$CMD tests/$MODULE"
fi

echo "Running: $CMD"

eval $CMD
