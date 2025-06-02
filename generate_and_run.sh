#!/bin/bash
# Generic Unreal Engine project generator and runner with ROS2 compatibility

set -e

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "${BLUE}===== Unreal Engine Project Generator and Runner =====${NC}"
}

print_success() {
    echo -e "${GREEN}[+] $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}[!] $1${NC}"
}

print_error() {
    echo -e "${RED}[!] $1${NC}"
}

# Setup
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_FILE=$(find "$PROJECT_DIR" -maxdepth 1 -name "*.uproject" | head -n 1)
PROJECT_NAME=$(basename "$PROJECT_FILE" .uproject)
CONFIG_FILE="$PROJECT_DIR/.ue_config"

if [ -z "$PROJECT_FILE" ]; then
    print_error "No .uproject file found in $PROJECT_DIR"
    exit 1
fi

print_success "Project directory: $PROJECT_DIR"
print_success "Detected project: $PROJECT_NAME"

# Locate UE directory
UE_DIR=""
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
    if [ ! -f "$UE_DIR/Engine/Build/BatchFiles/Linux/Build.sh" ]; then
        UE_DIR=""
    fi
fi

if [ -z "$UE_DIR" ]; then
    UE_LOCATIONS=(
        "$HOME/UnrealEngine"
        "$HOME/Epic/UnrealEngine"
        "$HOME/Unreal/Engine"
        "/opt/unreal-engine"
        "$HOME/UE_5.5"
        "$HOME/UnrealEngine-5.5.0"
    )
    for dir in "${UE_LOCATIONS[@]}"; do
        if [ -f "$dir/Engine/Build/BatchFiles/Linux/Build.sh" ]; then
            UE_DIR="$dir"
            break
        fi
    done
fi

if [ -z "$UE_DIR" ]; then
    print_warning "Could not auto-detect Unreal Engine directory"
    read -p "Please enter your Unreal Engine directory path: " UE_DIR
    if [ ! -f "$UE_DIR/Engine/Build/BatchFiles/Linux/Build.sh" ]; then
        print_error "Invalid Unreal Engine directory"
        exit 1
    fi
fi

echo "UE_DIR=\"$UE_DIR\"" > "$CONFIG_FILE"
print_success "Using Unreal Engine at: $UE_DIR"

# Argument parsing
EDITOR_MODE=true
BUILD_ONLY=false
MAP_NAME=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --no-editor)
            EDITOR_MODE=false; shift;;
        --build-only)
            BUILD_ONLY=true; shift;;
        --map)
            MAP_NAME="$2"; shift 2;;
        --help)
            echo "Usage: $0 [--no-editor] [--build-only] [--map NAME] [--help]"
            exit 0;;
        *)
            print_warning "Unknown option: $1"
            exit 1;;
    esac
done

# Menu
show_menu() {
    clear
    print_header
    echo "1) Generate project files"
    echo "2) Build and run editor"
    echo "3) Build and run standalone game"
    echo "4) Generate, build, and run editor"
    echo "5) Generate, build, and run game"
    echo "6) Exit"
    read -p "Choose [1-6]: " choice
}

generate_project_files() {
    print_success "Generating project files..."
    cd "$UE_DIR"
    Engine/Build/BatchFiles/Linux/GenerateProjectFiles.sh -project="$PROJECT_FILE" -game -engine
}

build_editor() {
    print_success "Building editor..."
    cd "$UE_DIR"
    Engine/Build/BatchFiles/Linux/Build.sh "${PROJECT_NAME}Editor" Linux Development -project="$PROJECT_FILE" -waitmutex
}

build_game() {
    print_success "Building game..."
    cd "$UE_DIR"
    Engine/Build/BatchFiles/Linux/Build.sh "$PROJECT_NAME" Linux Development -project="$PROJECT_FILE" -waitmutex
}

run_editor() {
    print_success "Launching Unreal Editor with map: $MAP_NAME"
    DDC_PATH="$HOME/.unrealengine/DerivedDataCache"
    mkdir -p "$DDC_PATH"
    "$UE_DIR/Engine/Binaries/Linux/UnrealEditor" "$PROJECT_FILE" "$MAP_NAME" -log -DDC=CreateInstalledDerivedDataBackendGraph -DDC=SharedDataCachePath="$DDC_PATH" -DDC-ForceMemoryCache
}

run_game() {
    print_success "Running standalone game with map: $MAP_NAME"
    "$PROJECT_DIR/Binaries/Linux/$PROJECT_NAME" "$MAP_NAME" -log
}

# Execution flow
if [ "$BUILD_ONLY" = true ] || [ "$EDITOR_MODE" = false ]; then
    generate_project_files
    if [ "$EDITOR_MODE" = true ]; then
        build_editor
    else
        build_game
    fi
    print_success "Build complete. Skipping launch as requested."
else
    show_menu
    case $choice in
        1)
            generate_project_files
            read -p "Run now? (y/n): " run_now
            if [[ $run_now =~ ^[Yy]$ ]]; then
                read -p "Editor or standalone? (e/s): " mode
                if [[ $mode =~ ^[Ee]$ ]]; then
                    build_editor
                    run_editor
                else
                    build_game
                    run_game
                fi
            fi;;
        2)
            build_editor
            run_editor;;
        3)
            build_game
            run_game;;
        4)
            generate_project_files
            build_editor
            run_editor;;
        5)
            generate_project_files
            build_game
            run_game;;
        6)
            print_success "Goodbye!"
            exit 0;;
        *)
            print_error "Invalid choice. Exiting."
            exit 1;;
    esac
fi

