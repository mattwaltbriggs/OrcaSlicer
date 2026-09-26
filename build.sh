#!/bin/bash
set -e

cd "$(dirname "$0")/build/arm64"

echo "Cleaning build intermediates..."
rm -rf build/
rm -rf ~/Library/Developer/Xcode/DerivedData/

AVAIL=$(df -g /Users/matthewbriggs/Documents/Code/Orca/ | awk 'NR==2 {print $4}')
echo "Free disk space: ${AVAIL}GB"

if [ "$AVAIL" -lt 15 ]; then
    echo "Warning: less than 15GB free. Build may fail on disk space."
    echo "Consider freeing space before continuing."
    exit 1
fi

echo "Building OrcaSlicer..."
xcodebuild -project OrcaSlicer.xcodeproj \
    -configuration RelWithDebInfo \
    -target OrcaSlicer \
    -jobs 8

echo ""
echo "Build complete: build/arm64/src/RelWithDebInfo/OrcaSlicer.app"

# Build MakerBotLib shared library (~3 seconds)
echo ""
echo "Building MakerBotLib.dylib..."
cd "$(dirname "$0")"
mkdir -p build_arm64

cc -std=c11 -O2 -fPIC -c deps_src/miniz/miniz.c -o build_arm64/miniz.o 2>/dev/null
c++ -std=c++17 -O2 -dynamiclib -fPIC -Wno-deprecated-literal-operator \
    -I deps_src \
    -I deps_src/miniz \
    -I src/MakerBotLib \
    -o build_arm64/libMakerBotLib.dylib \
    src/MakerBotLib/MakerBotLib.cpp build_arm64/miniz.o

# Copy to app bundle Frameworks
APP="build/arm64/src/RelWithDebInfo/OrcaSlicer.app"
if [ -d "$APP" ]; then
    mkdir -p "$APP/Contents/Frameworks"
    cp build_arm64/libMakerBotLib.dylib "$APP/Contents/Frameworks/"
    echo "Copied libMakerBotLib.dylib to $APP/Contents/Frameworks/"
fi

echo "All done."
