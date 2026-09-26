#!/bin/bash
# Quick rebuild of MakerBotLib.dylib only (~3 seconds)
# Run this after editing src/MakerBotLib/MakerBotLib.cpp
# No full OrcaSlicer rebuild needed.

set -e
cd "$(dirname "$0")"

mkdir -p build_arm64

cc -std=c11 -O2 -fPIC -c deps_src/miniz/miniz.c -o build_arm64/miniz.o 2>/dev/null
c++ -std=c++17 -O2 -dynamiclib -fPIC -Wno-deprecated-literal-operator \
    -I deps_src \
    -I deps_src/miniz \
    -I src/MakerBotLib \
    -o build_arm64/libMakerBotLib.dylib \
    src/MakerBotLib/MakerBotLib.cpp build_arm64/miniz.o

# Copy to app bundle
APP="build/arm64/src/RelWithDebInfo/OrcaSlicer.app"
if [ -d "$APP" ]; then
    mkdir -p "$APP/Contents/Frameworks"
    cp build_arm64/libMakerBotLib.dylib "$APP/Contents/Frameworks/"
    echo "Copied to $APP/Contents/Frameworks/"
fi

echo "Done."
