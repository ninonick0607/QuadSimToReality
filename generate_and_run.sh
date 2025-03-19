#!/bin/bash
# Script to generate project files and run Unreal Engine simulation
# for QuadSimToReality project with ROS2 support

# Exit on error
set -e

# Color codes for better readability
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}===== QuadSimToReality Project Generator and Runner =====${NC}"
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

# Get the project directory (where this script is located)
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
print_success "Project directory: $PROJECT_DIR"

# Config file to store UE directory
CONFIG_FILE="$PROJECT_DIR/.ue_config"

# Check if config file exists with UE_DIR
UE_DIR=""
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
    if [ -n "$UE_DIR" ] && [ -f "$UE_DIR/Engine/Build/BatchFiles/Linux/Build.sh" ]; then
        print_success "Found saved Unreal Engine directory: $UE_DIR"
    else
        UE_DIR=""
    fi
fi

# If not found in config, auto-detect
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
        if [ -d "$dir" ]; then
            if [ -f "$dir/Engine/Build/BatchFiles/Linux/Build.sh" ]; then
                UE_DIR="$dir"
                break
            fi
        fi
    done
fi

# If still not found, ask the user
if [ -z "$UE_DIR" ]; then
    print_warning "Could not auto-detect Unreal Engine directory"
    read -p "Please enter your Unreal Engine directory path: " UE_DIR
    
    if [ ! -f "$UE_DIR/Engine/Build/BatchFiles/Linux/Build.sh" ]; then
        print_error "Invalid Unreal Engine directory"
        exit 1
    fi
fi

# Save UE_DIR to config file
echo "UE_DIR=\"$UE_DIR\"" > "$CONFIG_FILE"
print_success "Using Unreal Engine at: $UE_DIR"
print_success "Engine location saved for future use"

# Default values
EDITOR_MODE=true
BUILD_ONLY=false
MAP_NAME="FlagMap"

# Parse command-line options
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-editor)
            EDITOR_MODE=false
            shift
            ;;
        --build-only)
            BUILD_ONLY=true
            shift
            ;;
        --map)
            MAP_NAME="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --no-editor    Launch in standalone game mode"
            echo "  --build-only   Only build, don't launch"
            echo "  --map NAME     Specify map to load (default: FlagMap)"
            echo "  --help         Show this help message"
            exit 0
            ;;
        *)
            print_warning "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Function to display menu
show_menu() {
    clear
    print_header
    echo "What would you like to do?"
    echo "1) Generate project files"
    echo "2) Build and run editor"
    echo "3) Build and run standalone game"
    echo "4) Generate project files, build and run editor"
    echo "5) Generate project files, build and run standalone game"
    echo "6) Exit"
    echo
    read -p "Enter your choice [1-6]: " choice
}

# Function to generate project files
generate_project_files() {
    print_success "Generating project files..."
    cd "$UE_DIR"
    Engine/Build/BatchFiles/Linux/GenerateProjectFiles.sh -project="$PROJECT_DIR/QuadSimToReality.uproject" -game -engine
    print_success "Project files generated successfully!"
}

# Function to build editor
build_editor() {
    print_success "Building editor..."
    cd "$UE_DIR"
    Engine/Build/BatchFiles/Linux/Build.sh QuadSimToRealityEditor Linux Development -project="$PROJECT_DIR/QuadSimToReality.uproject" -waitmutex
    print_success "Editor built successfully!"
}

# Function to build game
build_game() {
    print_success "Building game..."
    cd "$UE_DIR"
    Engine/Build/BatchFiles/Linux/Build.sh QuadSimToReality Linux Development -project="$PROJECT_DIR/QuadSimToReality.uproject" -waitmutex
    print_success "Game built successfully!"
}

# Function to run editor
run_editor() {
    print_success "Launching Unreal Editor with map: $MAP_NAME"
    "$UE_DIR/Engine/Binaries/Linux/UnrealEditor" "$PROJECT_DIR/QuadSimToReality.uproject" "$MAP_NAME" -log
}

# Function to run game
run_game() {
    print_success "Launching standalone game with map: $MAP_NAME"
    "$PROJECT_DIR/Binaries/Linux/QuadSimToReality" "$MAP_NAME" -log
}

# Check if command line options were provided
if [ "$BUILD_ONLY" = true ] || [ "$EDITOR_MODE" = false ]; then
    # Command line options were provided, skip the menu
    if [ "$BUILD_ONLY" = true ]; then
        generate_project_files
        if [ "$EDITOR_MODE" = true ]; then
            build_editor
        else
            build_game
        fi
        print_success "Build completed. Not launching as requested."
    else
        generate_project_files
        if [ "$EDITOR_MODE" = true ]; then
            build_editor
            run_editor
        else
            build_game
            run_game
        fi
    fi
else
    # No command line options, show the menu
    show_menu
    
    case $choice in
        1)
            generate_project_files
            read -p "Would you like to build and run now? (y/n): " run_now
            if [[ $run_now == "y" || $run_now == "Y" ]]; then
                read -p "Run in editor mode or standalone? (e/s): " mode
                if [[ $mode == "e" || $mode == "E" ]]; then
                    build_editor
                    run_editor
                elif [[ $mode == "s" || $mode == "S" ]]; then
                    build_game
                    run_game
                else
                    print_warning "Invalid option. Exiting."
                fi
            else
                print_success "Project files generated. Exiting."
            fi
            ;;
        2)
            build_editor
            run_editor
            ;;
        3)
            build_game
            run_game
            ;;
        4)
            generate_project_files
            build_editor
            run_editor
            ;;
        5)
            generate_project_files
            build_game
            run_game
            ;;
        6)
            print_success "Goodbye!"
            exit 0
            ;;
        *)
            print_error "Invalid choice. Exiting."
            exit 1
            ;;
    esac
fi
