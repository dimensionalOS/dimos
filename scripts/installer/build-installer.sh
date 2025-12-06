#!/usr/bin/env bash
# Build installer script for DimOS Ubuntu installer image
# This script provides modular functions for building the installer image

set -euo pipefail

# ============================================================================
# Configuration & Constants
# ============================================================================

# Colors for output (following patterns from bin/lfs_push.sh)
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m' # No Color

# ============================================================================
# Utility Functions: Logging
# ============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $*"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $*"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $*"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $*" >&2
}

# ============================================================================
# Utility Functions: Argument Parsing
# ============================================================================

# Parse arguments and store in associative array (requires bash 4+)
declare -A ARGS

parse_args() {
    local key=""
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --*)
                # If we have a pending key from previous iteration, it's a boolean flag
                if [[ -n "$key" ]]; then
                    ARGS["$key"]="true"
                fi
                key="${1#--}"
                # Handle --key=value format
                if [[ "$key" == *"="* ]]; then
                    local k="${key%%=*}"
                    local v="${key#*=}"
                    ARGS["$k"]="$v"
                    key=""
                else
                    key="$key"
                fi
                ;;
            *)
                if [[ -n "$key" ]]; then
                    ARGS["$key"]="$1"
                    key=""
                else
                    log_error "Unexpected argument: $1"
                    return 1
                fi
                ;;
        esac
        shift
    done
    # Handle trailing boolean flag
    if [[ -n "$key" ]]; then
        ARGS["$key"]="true"
    fi
}

get_arg() {
    local key="$1"
    echo "${ARGS[$key]:-}"
}

require_arg() {
    local key="$1"
    local value="${ARGS[$key]:-}"
    if [[ -z "$value" ]]; then
        log_error "Required argument --$key is missing"
        return 1
    fi
    echo "$value"
}

# ============================================================================
# Utility Functions: GITHUB_OUTPUT Handling
# ============================================================================

# Sanitize a value for GITHUB_OUTPUT
# Handles newlines, carriage returns, and percent signs
sanitize_github_output() {
    local value="$1"
    # Replace % with %25
    value="${value//'%'/'%25'}"
    # Replace newlines with %0A
    value="${value//$'\n'/'%0A'}"
    # Replace carriage returns with %0D
    value="${value//$'\r'/'%0D'}"
    echo "$value"
}

# Write key=value to GITHUB_OUTPUT with sanitization
write_github_output() {
    local key="$1"
    local value="$2"
    local sanitized
    sanitized=$(sanitize_github_output "$value")
    echo "${key}=${sanitized}" >> "${GITHUB_OUTPUT:-/dev/stdout}"
}

# ============================================================================
# Utility Functions: Validation
# ============================================================================

validate_path() {
    local path="$1"
    local description="${2:-Path}"
    if [[ ! -e "$path" ]]; then
        log_error "$description does not exist: $path"
        return 1
    fi
    return 0
}

# Check if a step should be skipped
# Usage: should_skip_step "docker-save" "$skip_steps"
should_skip_step() {
    local step="$1"
    local skip_steps="$2"

    if [[ -z "$skip_steps" ]]; then
        return 1  # Don't skip if no skip list provided
    fi

    # Parse comma-separated list and check if step is in it
    IFS=',' read -ra skip_array <<< "$skip_steps"
    for skip_item in "${skip_array[@]}"; do
        # Trim whitespace
        skip_item="${skip_item#"${skip_item%%[![:space:]]*}"}"
        skip_item="${skip_item%"${skip_item##*[![:space:]]}"}"
        if [[ "$skip_item" == "$step" ]]; then
            return 0  # Step should be skipped
        fi
    done

    return 1  # Step should not be skipped
}

check_command() {
    local cmd="$1"
    if ! command -v "$cmd" >/dev/null 2>&1; then
        log_error "Command not found: $cmd"
        return 1
    fi
    return 0
}

# ============================================================================
# Utility Functions: Docker Operations
# ============================================================================

docker_pull_image() {
    local image="$1"
    log_info "Pulling Docker image: $image"
    docker pull "$image" || {
        log_error "Failed to pull Docker image: $image"
        return 1
    }
}

docker_save_images() {
    local output_file="$1"
    shift
    local images=("$@")
    log_info "Saving Docker images to: $output_file"
    docker save "${images[@]}" | gzip > "$output_file" || {
        log_error "Failed to save Docker images"
        return 1
    }
    log_success "Saved $(du -h "$output_file" | cut -f1) of Docker images"
}

# ============================================================================
# Utility Functions: File/Archive Operations
# ============================================================================

create_tarball() {
    local output_file="$1"
    local source_dir="$2"
    local exclude_patterns=("${@:3}")

    log_info "Creating tarball: $output_file"
    log_info "Source directory: $source_dir"

    local exclude_args=()
    for pattern in "${exclude_patterns[@]}"; do
        exclude_args+=(--exclude="$pattern")
    done

    tar -czf "$output_file" \
        "${exclude_args[@]}" \
        --checkpoint=1000 \
        --checkpoint-action=dot \
        -C "$(dirname "$source_dir")" \
        "$(basename "$source_dir")" || {
        log_error "Failed to create tarball"
        return 1
    }

    log_success "Created tarball: $(du -h "$output_file" | cut -f1)"
}

calculate_checksum() {
    local file="$1"
    local algorithm="${2:-sha256}"

    case "$algorithm" in
        sha256)
            sha256sum "$file" | cut -d' ' -f1
            ;;
        md5)
            md5sum "$file" | cut -d' ' -f1
            ;;
        *)
            log_error "Unsupported checksum algorithm: $algorithm"
            return 1
            ;;
    esac
}

# ============================================================================
# Action Verb: prepare-dimos-img-components
# ============================================================================

action_prepare_dimos_img_components() {
    local docker_image_tag
    local staging_dir
    local workspace
    local skip_steps

    docker_image_tag=$(require_arg "docker-image-tag") || return 1
    staging_dir=$(require_arg "staging-dir") || return 1
    workspace=$(require_arg "workspace") || return 1
    skip_steps=$(get_arg "skip-steps")

    # Validate inputs
    validate_path "$workspace" "Workspace directory" || return 1

    # Create staging directory structure
    mkdir -p "$staging_dir/docker-images"

    # Pull Docker images (skip if docker-pull is in skip list)
    local images=(
        "ghcr.io/dimensionalos/ros:$docker_image_tag"
        "ghcr.io/dimensionalos/python:$docker_image_tag"
        "ghcr.io/dimensionalos/dev:$docker_image_tag"
    )
    local pulled_images=()

    if ! should_skip_step "docker-pull" "$skip_steps"; then
        for image in "${images[@]}"; do
            if docker_pull_image "$image"; then
                pulled_images+=("$image")
            fi
        done
    else
        # If skipping pull, check for locally available images
        for image in "${images[@]}"; do
            if docker image inspect "$image" >/dev/null 2>&1; then
                pulled_images+=("$image")
            fi
        done
    fi

    # Save Docker images (skip if docker-save is in skip list)
    local docker_images_file=""
    if ! should_skip_step "docker-save" "$skip_steps"; then
    if [[ ${#pulled_images[@]} -eq 0 ]]; then
            log_error "No Docker images available for saving"
        return 1
    fi

        docker_images_file="$staging_dir/docker-images/dimos-images-$docker_image_tag.tar.gz"
    docker_save_images "$docker_images_file" "${pulled_images[@]}"
    fi

    # Write output paths to GITHUB_OUTPUT if available
    if [[ -n "${GITHUB_OUTPUT:-}" ]]; then
        if ! should_skip_step "docker-save" "$skip_steps" && [[ -n "$docker_images_file" ]]; then
        write_github_output "docker-images-path" "$docker_images_file"
        fi
    fi

    log_success "Components prepared in: $staging_dir"
}

# ============================================================================
# Action Verb: install-tools
# ============================================================================

# Verify a package installation using the mapping
verify_package() {
    local pkg="$1"
    local verification_method="${2:-}"

    case "$verification_method" in
        "file:"*)
            # File-based verification (e.g., "file:/usr/lib/grub/i386-pc/normal.mod")
            local file_path="${verification_method#file:}"
            if [[ -f "$file_path" ]]; then
                return 0
            else
                log_warn "Package $pkg: expected file not found: $file_path"
                return 1
            fi
            ;;
        *)
            # Command-based verification (default)
            if check_command "$verification_method"; then
                return 0
            else
                log_warn "Package $pkg: command '$verification_method' not found in PATH"
                return 1
            fi
            ;;
    esac
}

action_install_tools() {
    local skip_steps
    skip_steps=$(get_arg "skip-steps")

    if [[ $EUID -ne 0 ]]; then
        log_warn "This action-verb requires root privileges. Some operations may fail if not running as root."
    fi

    # Package name -> verification method mapping
    # Format: "command_name" or "file:/path/to/file"
    declare -A packages=(
        ["xorriso"]="xorriso"
        ["grub-pc-bin"]="file:/usr/lib/grub/i386-pc/normal.mod"
        ["grub-efi-amd64-bin"]="file:/usr/lib/grub/x86_64-efi/normal.mod"
        ["parted"]="parted"
        ["gdisk"]="gdisk"
        ["squashfs-tools"]="mksquashfs"
        ["gzip"]="gzip"
        ["xz-utils"]="xz"
        ["wget"]="wget"
        ["curl"]="curl"
        ["tar"]="tar"
        ["rsync"]="rsync"
        ["npm"]="npm"
        ["git-lfs"]="git-lfs"
        ["golang-go"]="go"
    )

    # Update package lists (skip if update is in skip list)
    if ! should_skip_step "update" "$skip_steps"; then
        apt-get update -qq || {
            log_error "Failed to update package lists"
            return 1
        }
    fi

    # Install packages (skip if install is in skip list)
    if ! should_skip_step "install" "$skip_steps"; then
        apt-get install -y -qq "${!packages[@]}" || {
            log_error "Failed to install required packages"
            return 1
        }
    fi

    # Verify installations using mapping (skip if verify is in skip list)
    if ! should_skip_step "verify" "$skip_steps"; then
        local failed_verifications=0
        for pkg in "${!packages[@]}"; do
            local verification_method="${packages[$pkg]}"
            if ! verify_package "$pkg" "$verification_method"; then
                failed_verifications=$((failed_verifications + 1))
            fi
        done

        if [[ $failed_verifications -gt 0 ]]; then
            log_warn "$failed_verifications package(s) failed verification (may still be functional)"
        fi
    fi

    log_success "Required tools installed"
}

# ============================================================================
# Action Verb: install-devcontainer-cli
# ============================================================================

action_install_devcontainer_cli() {
    local workspace
    workspace=$(require_arg "workspace") || return 1
    validate_path "$workspace" "Workspace directory" || return 1

    local bin_dir="$workspace/bin"
    if [[ ! -d "$bin_dir" ]]; then
        log_error "bin directory not found: $bin_dir"
        return 1
    fi

    if [[ -d "$bin_dir/node_modules" ]]; then
        log_success "devcontainer CLI already installed"
        return 0
    fi

    # Initialize package.json if it doesn't exist
    if [[ ! -f "$bin_dir/package.json" ]]; then
        npm init --prefix "$bin_dir" -y 1>/dev/null || {
            log_error "Failed to initialize npm package.json"
            return 1
        }
    fi

    # Install @devcontainers/cli using --prefix to avoid cd
    npm install --prefix "$bin_dir" @devcontainers/cli 1>&2 || {
        log_error "Failed to install @devcontainers/cli"
        return 1
    }

    log_success "devcontainer CLI installed successfully"
}

# ============================================================================
# Action Verb: download-ubuntu-image-tool
# ============================================================================

action_download_ubuntu_image_tool() {
    local staging_dir
    staging_dir=$(require_arg "staging-dir") || return 1

    # Check if git is installed
    if ! command -v git >/dev/null 2>&1; then
        log_error "git is not installed. Please install git first."
        return 1
    fi

    local ubuntu_image_dir="$staging_dir/ubuntu-image-tool-git"

    # Check if directory already exists
    if [[ -d "$ubuntu_image_dir" ]]; then
        if [[ -d "$ubuntu_image_dir/.git" ]]; then
            log_info "ubuntu-image repository already exists at: $ubuntu_image_dir"
            return 0
        else
            log_warn "Directory exists but is not a git repository: $ubuntu_image_dir"
            rm -rf "$ubuntu_image_dir"
        fi
    fi

    # Create parent directory if it doesn't exist
    mkdir -p "$staging_dir" || {
        log_error "Failed to create staging directory: $staging_dir"
        return 1
    }

    log_info "Cloning ubuntu-image repository to: $ubuntu_image_dir"
    git clone https://git.launchpad.net/ubuntu-image "$ubuntu_image_dir" || {
        log_error "Failed to clone ubuntu-image repository"
        return 1
    }

    log_success "ubuntu-image repository cloned successfully to: $ubuntu_image_dir"
}

# ============================================================================
# Action Verb: install-git-lfs-files
# ============================================================================

action_install_git_lfs_files() {
    local workspace
    workspace=$(require_arg "workspace") || return 1
    validate_path "$workspace" "Workspace directory" || return 1

    # Check if git-lfs is installed
    if ! command -v git-lfs >/dev/null 2>&1; then
        log_error "git-lfs is not installed. Please run install-tools first."
        return 1
    fi

    cd "$workspace" || {
        log_error "Failed to change to workspace directory"
        return 1
    }

    # Check if this is a git repository
    if [[ ! -d ".git" ]]; then
        log_error "Not a git repository: $workspace"
        return 1
    fi

    # Configure git-lfs
    git lfs install || {
        log_error "Failed to configure git-lfs"
        return 1
    }
    git lfs fetch || true
    git lfs checkout || true
    log_success "Git LFS files installed successfully"
}

# ============================================================================
# Action Verb: substitute-tokens
# ============================================================================

action_substitute_tokens() {
    # Substitutes a token in a YAML file with a replacement string.
    # Token format: <{ascii_identifier-with.underscores_periods-and-dots}>
    # The token must be enclosed in <{ and }> and contain only ASCII letters,
    # numbers, underscores, periods, and hyphens.
    local input_file
    local input_token
    local substitute_string

    input_file=$(require_arg "input-file") || return 1
    input_token=$(require_arg "input-token") || return 1
    substitute_string=$(get_arg "substitute-string")
    if [[ -z "$substitute_string" ]]; then
        log_error "Required argument --substitute-string is missing"
        return 1
    fi

    # Validate inputs
    validate_path "$input_file" "Input file" || return 1

    # Check if file exists and is readable
    if [[ ! -r "$input_file" ]]; then
        log_error "Input file is not readable: $input_file"
        return 1
    fi

    # Create temporary file for in-place editing
    local temp_file
    temp_file=$(mktemp) || {
        log_error "Failed to create temporary file"
        return 1
    }

    # Perform substitution using sed
    # Token format is always: <{ascii_identifier-with.underscores_periods-and-dots}>
    # In sed BRE, use character classes for { and } to match them literally
    # < and > are literal characters and should not be escaped
    local escaped_token
    escaped_token=$(printf '%s\n' "$input_token" | sed -e 's/\\/\\\\/g' \
        -e 's/{/[{]/g' \
        -e 's/}/[}]/g' \
        -e 's/\./\\./g')

    local escaped_substitute
    escaped_substitute=$(printf '%s\n' "$substitute_string" | sed -e 's/\\/\\\\/g' \
        -e 's/\./\\./g' \
        -e 's/\*/\\*/g' \
        -e 's/\^/\\^/g' \
        -e 's/\$/\\$/g' \
        -e 's/\[/\\[/g' \
        -e 's/\]/\\]/g' \
        -e 's/(/\\(/g' \
        -e 's/)/\\)/g' \
        -e 's/+/\\+/g' \
        -e 's/?/\\?/g' \
        -e 's/{/\\{/g' \
        -e 's/}/\\}/g' \
        -e 's/|/\\|/g' \
        -e 's|/|\\/|g')

    if sed "s|${escaped_token}|${escaped_substitute}|g" "$input_file" > "$temp_file"; then
        # Replace original file with modified version
        if mv "$temp_file" "$input_file"; then
            log_success "Token substitution completed successfully"
        else
            log_error "Failed to replace original file"
            rm -f "$temp_file"
            return 1
        fi
    else
        log_error "Failed to perform substitution"
        rm -f "$temp_file"
        return 1
    fi
}

# ============================================================================
# Action Verb: determine-path-relative-to
# ============================================================================

action_determine_path_relative_to() {
    local anchor_path
    local path_to_be_relativized

    anchor_path=$(require_arg "anchor-path") || return 1
    path_to_be_relativized=$(require_arg "path-to-be-relativized") || return 1

    # Validate inputs
    if [[ ! -e "$anchor_path" ]] && [[ ! -d "$(dirname "$anchor_path")" ]]; then
        log_error "Anchor path does not exist and parent directory does not exist: $anchor_path"
        return 1
    fi

    if [[ ! -e "$path_to_be_relativized" ]] && [[ ! -d "$(dirname "$path_to_be_relativized")" ]]; then
        log_error "Path to be relativized does not exist and parent directory does not exist: $path_to_be_relativized"
        return 1
    fi

    # If anchor_path is a file, use its directory; if it's a directory, use it as-is
    local anchor_dir
    if [[ -f "$anchor_path" ]]; then
        anchor_dir="$(dirname "$anchor_path")"
    elif [[ -d "$anchor_path" ]]; then
        anchor_dir="$anchor_path"
    else
        # Path doesn't exist, assume it's a file and use its directory
        anchor_dir="$(dirname "$anchor_path")"
    fi

    # Calculate relative path using realpath
    if ! command -v realpath >/dev/null 2>&1; then
        log_error "realpath command not found. Please install coreutils."
        return 1
    fi

    local relative_path
    relative_path=$(realpath --relative-to="$anchor_dir" "$path_to_be_relativized" 2>/dev/null)
    if [[ $? -ne 0 ]]; then
        log_error "Failed to calculate relative path using realpath"
        return 1
    fi

    # Output the relative path (for use in scripts/pipelines)
    echo "$relative_path"

    # Also write to GITHUB_OUTPUT if available
    if [[ -n "${GITHUB_OUTPUT:-}" ]]; then
        write_github_output "relative-path" "$relative_path"
    fi
}

# ============================================================================
# Action Verb: build-img
# ============================================================================

action_build_img() {
    local docker_image_tag
    local staging_dir
    local workspace
    local sha
    local ubuntu_image_tool_dir
    local log_file
    local skip_steps
    local clean_staging_first

    docker_image_tag=$(require_arg "docker-image-tag") || return 1
    staging_dir=$(require_arg "staging-dir") || return 1
    workspace=$(require_arg "workspace") || return 1
    sha=$(require_arg "sha") || return 1
    ubuntu_image_tool_dir=$(require_arg "ubuntu-image-tool-dir") || return 1
    log_file=$(get_arg "log-file")
    skip_steps=$(get_arg "skip-steps")
    clean_staging_first=$(get_arg "clean-ubuntu-image-tool-staging-first")

    # Check for root privileges
    if [[ $EUID -ne 0 ]]; then
        log_warn "This action-verb requires root privileges. The build may fail if not running as root."
    fi

    validate_path "$workspace" "Workspace directory" || return 1
    validate_path "$ubuntu_image_tool_dir" "Ubuntu-image tool directory" || return 1
    # Validate that ubuntu-image repository has the expected code in it
    if [[ ! -f "$ubuntu_image_tool_dir/cmd/ubuntu-image/main.go" ]]; then
        log_error "Invalid ubuntu-image tool directory: $ubuntu_image_tool_dir (missing cmd/ubuntu-image/main.go)"
        return 1
    fi

    validate_path "$staging_dir/docker-images" "Docker images directory" || return 1
    # YAML file path relative to workspace
    local yaml_file="$workspace/scripts/installer/ubuntu-desktop-installer-amd64.yaml"
    validate_path "$yaml_file" "YAML definition file" || return 1

    # Work directory for ubuntu-image (subdir of staging dir)
    local workdir="$staging_dir/ubuntu-image-tool-workdir"
    # Output directory for ubuntu-image (separate from workdir)
    local outputdir="$staging_dir/ubuntu-image-tool-output"

    # Clean staging directories if requested
    if [[ -n "$clean_staging_first" ]]; then
        if [[ -d "$workdir" ]]; then
            find "$workdir" -mindepth 1 -delete || true
        fi

        if [[ -d "$outputdir" ]]; then
            find "$outputdir" -mindepth 1 -delete || true
        fi
    fi

    for dir in "$workdir" "$outputdir"; do
        mkdir -p "$dir" || {
            log_error "Failed to create directory: $dir"
            return 1
        }
    done

    log_info "Docker image tag: $docker_image_tag"
    log_info "Commit SHA: $sha"
    log_info "Staging directory: $staging_dir"
    log_info "Work directory: $workdir"
    log_info "Output directory: $outputdir"
    log_info "YAML file: $yaml_file"

    # Build ubuntu-image tool if not already built (skip if compile is in skip list)
    local ubuntu_image_binary="$ubuntu_image_tool_dir/ubuntu-image"
    if should_skip_step "compile" "$skip_steps"; then
        if [[ ! -f "$ubuntu_image_binary" ]]; then
            log_error "ubuntu-image binary not found and compile step is skipped: $ubuntu_image_binary"
            return 1
        fi
    else
        if [[ ! -f "$ubuntu_image_binary" ]] || [[ "$ubuntu_image_tool_dir/cmd/ubuntu-image/main.go" -nt "$ubuntu_image_binary" ]]; then
            cd "$ubuntu_image_tool_dir" || {
                log_error "Failed to change to ubuntu-image tool directory"
                return 1
            }

            if ! command -v go >/dev/null 2>&1; then
                log_error "Go compiler not found. Please install golang-go"
                return 1
            fi

            go build -o "$ubuntu_image_binary" ./cmd/ubuntu-image || {
                log_error "Failed to build ubuntu-image tool"
                return 1
            }
            cd - > /dev/null
        fi
    fi

    # Construct the ubuntu-image command invocation
    local ubuntu_image_cmd=(
        "$ubuntu_image_binary"
        "classic"
        "-w" "$workdir"
        "-O" "$outputdir"
        "$yaml_file"
        "--debug"
    )

    # Execute ubuntu-image command (skip if tool-call is in skip list)
    if ! should_skip_step "tool-call" "$skip_steps"; then
        # Execute the command, redirecting output to log file if specified
        if [[ -n "$log_file" ]]; then
            if ! "${ubuntu_image_cmd[@]}" 2>&1 | tee "$log_file"; then
                log_error "ubuntu-image build failed. Check log file: $log_file"
                return 1
            fi
        else
            if ! "${ubuntu_image_cmd[@]}"; then
                log_error "ubuntu-image build failed"
                return 1
            fi
        fi
    fi

    # Find the generated .img file in the output directory
    # ubuntu-image creates the image in the output directory specified by -O
    local img_file
    img_file=$(find "$outputdir" -name "*.img" -type f | head -n1)
    if [[ -z "$img_file" ]]; then
        log_error "No .img file found in output directory: $outputdir"
        return 1
    fi

    # Generate checksums (skip if sha-generation is in skip list)
    local sha256_file="${img_file}.sha256"
    local md5_file="${img_file}.md5"

    if ! should_skip_step "sha-generation" "$skip_steps"; then
        calculate_checksum "$img_file" sha256 > "$sha256_file"
        calculate_checksum "$img_file" md5 > "$md5_file"
    fi

    # Rename output files with date+time suffix
    local datetime_suffix
    datetime_suffix=$(date +"%Y%m%d-%H%M%S")

    local img_dir
    local img_basename
    local img_name_no_ext
    local img_ext
    img_dir=$(dirname "$img_file")
    img_basename=$(basename "$img_file")
    img_name_no_ext="${img_basename%.*}"
    img_ext="${img_basename##*.}"

    local new_img_file="${img_dir}/${img_name_no_ext}-${datetime_suffix}.${img_ext}"
    mv "$img_file" "$new_img_file" || {
        log_error "Failed to rename .img file"
        return 1
    }

    # Rename checksum files if they exist (they will include the datetime suffix)
    # The hash files are renamed to match the new .img filename which includes the datetime
    if [[ -f "$sha256_file" ]]; then
        local new_sha256_file="${new_img_file}.sha256"
        mv "$sha256_file" "$new_sha256_file" 2>/dev/null || true
    fi

    if [[ -f "$md5_file" ]]; then
        local new_md5_file="${new_img_file}.md5"
        mv "$md5_file" "$new_md5_file" 2>/dev/null || true
    fi

    # Also rename .manifest and .filelist files if they exist
    local manifest_file="${img_dir}/dimos-desktop-installer-amd64.manifest"
    if [[ -f "$manifest_file" ]]; then
        local new_manifest_file="${img_dir}/dimos-desktop-installer-amd64-${datetime_suffix}.manifest"
        mv "$manifest_file" "$new_manifest_file" 2>/dev/null || true
    fi

    local filelist_file="${img_dir}/dimos-desktop-installer-amd64.filelist"
    if [[ -f "$filelist_file" ]]; then
        local new_filelist_file="${img_dir}/dimos-desktop-installer-amd64-${datetime_suffix}.filelist"
        mv "$filelist_file" "$new_filelist_file" 2>/dev/null || true
    fi

    # Update img_file variable to point to the renamed file
    img_file="$new_img_file"

    # Output the .img file path to GITHUB_OUTPUT for use in workflows
    if [[ -n "${GITHUB_OUTPUT:-}" ]]; then
        write_github_output "img-file" "$new_img_file"
    fi

    log_success "Installer image created: $img_file"
}

# ============================================================================
# Action: Cleanup Storage
# ============================================================================

# Get current disk usage percentage
get_disk_usage() {
    df / | awk 'NR==2 {print $5}' | sed 's/%//'
}

# Common function to clean up installer .img files and staging directories
# Arguments:
#   $1: age_days - minimum age in days for files to be deleted (e.g., 30, 7, 1)
#   $2: description - human-readable description for logging (e.g., "30 days", "7 days", "24 hours")
#   $3: staging_dir - required staging directory to clean
# Returns: number of items cleaned up
cleanup_installer_files_by_age() {
    local age_days="$1"
    local description="$2"
    local staging_dir="$3"

    if [[ -z "$staging_dir" || ! -d "$staging_dir" ]]; then
        log_error "Valid staging_dir argument is required"
        return 1
    fi

    # Build find command with optional age filter
    # If age_days is empty or 0, don't use -mtime filter (clean all)
    local mtime_filter=""
    if [[ -n "$age_days" && "$age_days" != "0" ]]; then
        mtime_filter="-mtime +$age_days"
    fi

    # Remove all files in ubuntu-image-tool-output subdirectory that match age criteria
    local outputdir="$staging_dir/ubuntu-image-tool-output"
    if [[ -d "$outputdir" ]]; then
        find "$outputdir" -maxdepth 1 -type f $mtime_filter -delete 2>/dev/null || true
    fi

    # Clean ubuntu-image workdir
    local workdir="$staging_dir/ubuntu-image-tool-workdir"
    if [[ -d "$workdir" ]]; then
        find "$workdir" -mindepth 1 $mtime_filter -delete 2>/dev/null || true
    fi

    # Clean docker images directory
    local docker_images_dir="$staging_dir/docker-images"
    if [[ -d "$docker_images_dir" ]]; then
        find "$docker_images_dir" -mindepth 1 $mtime_filter -delete 2>/dev/null || true
    fi
}

action_cleanup_storage() {
    local staging_dir
    local has_routine=false
    local has_usage_60=false
    local has_usage_85=false
    local has_clean_all=false

    staging_dir=$(require_arg "staging-dir") || return 1

    # Check if boolean flags are present
    # Check if keys exist in ARGS (boolean flags will have value "true")
    [[ -n "${ARGS[routine-cleanup]:-}" ]] && has_routine=true
    [[ -n "${ARGS[usage-above-60]:-}" ]] && has_usage_60=true
    [[ -n "${ARGS[usage-above-85]:-}" ]] && has_usage_85=true
    [[ -n "${ARGS[clean-all]:-}" ]] && has_clean_all=true

    # At least one option must be provided
    if [[ "$has_routine" == false && "$has_usage_60" == false && "$has_usage_85" == false && "$has_clean_all" == false ]]; then
        log_error "At least one cleanup option must be specified: --routine-cleanup, --usage-above-60, --usage-above-85, or --clean-all"
        return 1
    fi

    # Run clean-all if requested (takes precedence)
    if [[ "$has_clean_all" == true ]]; then
        cleanup_installer_files_by_age "" "" "$staging_dir"
    else
        # Run routine cleanup if requested
        if [[ "$has_routine" == true ]]; then
            cleanup_installer_files_by_age 30 "30 days" "$staging_dir"
        fi
        # Run 60% cleanup if requested
        if [[ "$has_usage_60" == true ]]; then
            cleanup_installer_files_by_age 7 "7 days" "$staging_dir"
        fi
        # Run 85% cleanup if requested
        if [[ "$has_usage_85" == true ]]; then
            cleanup_installer_files_by_age 1 "24 hours" "$staging_dir"
        fi
    fi

    log_success "Storage cleanup completed"
}

# ============================================================================
# Main Dispatcher
# ============================================================================

show_usage() {
    cat <<EOF
Usage: $0 <action> [options]

Actions:
  prepare-dimos-img-components      Prepare tarballs and components for installer
    --docker-image-tag <tag>        Docker image tag to use
    --staging-dir <dir>             Staging directory for components
    --workspace <dir>               Workspace directory with source code
    --skip-steps <list>             Optional: Comma-separated list of steps to skip
                                    (available: docker-pull, docker-save)

  install-tools                     Install required system packages
    --skip-steps <list>              Optional: Comma-separated list of steps to skip
                                    (available: update, install, verify)

  download-ubuntu-image-tool        Clone ubuntu-image repository from Launchpad
    --staging-dir <dir>              Staging directory where ubuntu-image-tool-git will be created

  install-git-lfs-files             Install Git LFS files (fetch and checkout)
    --workspace <dir>                Workspace directory

  install-devcontainer-cli          Install @devcontainers/cli npm package in bin directory
    --workspace <dir>                Workspace directory

  substitute-tokens                 Substitute tokens in YAML file
    --input-file <file>             Input YAML file to modify
    --input-token <token>           Token to replace (e.g., "<{token-name}>")
    --substitute-string <string>    String to substitute for the token

  determine-path-relative-to        Calculate relative path from anchor to target
    --anchor-path <path>            Anchor path (file or directory) to calculate relative to
    --path-to-be-relativized <path> Path to make relative to anchor

  build-img                         Build the Ubuntu installer image using ubuntu-image
    --docker-image-tag <tag>        Docker image tag
    --staging-dir <dir>             Staging directory
    --workspace <dir>               Workspace directory
    --sha <commit_sha>              Git commit SHA
    --ubuntu-image-tool-dir <dir>   Directory containing ubuntu-image source tree
    --log-file <file>              Optional: Path to log file for capturing output
    --skip-steps <list>            Optional: Comma-separated list of steps to skip
                                    (available: compile, tool-call, sha-generation)
    --clean-ubuntu-image-tool-staging-first
                                    Optional: Clean contents of workdir and outputdir before building

  cleanup-storage                   Clean up installer .img files and staging directories
    --staging-dir <dir>             Required: Staging directory to clean
    --routine-cleanup               Run routine cleanup (remove .img files older than 30 days)
    --usage-above-60                Run cleanup when usage > 60% (remove .img files older than 7 days)
    --usage-above-85                Run aggressive cleanup when usage > 85% (remove .img files older than 24 hours)
    --clean-all                     Remove all files regardless of age (takes precedence over other options)

Examples:
  $0 prepare-dimos-img-components --docker-image-tag latest --staging-dir /tmp/staging --workspace /workspace
  $0 install-tools
  $0 install-git-lfs-files --workspace /workspace
  $0 install-devcontainer-cli --workspace /workspace
  $0 build-img --docker-image-tag latest --staging-dir /tmp/staging --workspace /workspace --sha abc123
EOF
}

main() {
    if [[ $# -eq 0 ]]; then
        show_usage
        exit 0
    fi

    local action="$1"
    shift

    # Parse remaining arguments
    parse_args "$@"

    # Route to appropriate action function
    case "$action" in
        prepare-dimos-img-components)
            action_prepare_dimos_img_components
            ;;
        install-tools)
            action_install_tools
            ;;
        download-ubuntu-image-tool)
            action_download_ubuntu_image_tool
            ;;
        install-git-lfs-files)
            action_install_git_lfs_files
            ;;
        install-devcontainer-cli)
            action_install_devcontainer_cli
            ;;
        substitute-tokens)
            action_substitute_tokens
            ;;
        determine-path-relative-to)
            action_determine_path_relative_to
            ;;
        build-img)
            action_build_img
            ;;
        cleanup-storage)
            action_cleanup_storage
            ;;
        *)
            log_error "Unknown action: $action"
            echo ""
            show_usage
            exit 1
            ;;
    esac
}

# Run main if script is executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
