#!/usr/bin/env bash
# 构建 arm64-v8a Release APK：
#   需要：Rust(+ aarch64-linux-android target)、cargo-ndk、Android SDK(platform 34, build-tools 34)、NDK r27、JDK 17+/21
#   环境：ANDROID_HOME / ANDROID_NDK_HOME / JAVA_HOME
# 产物：android/app/build/outputs/apk/release/app-release.apk
set -euo pipefail
cd "$(dirname "$0")/.."
: "${ANDROID_NDK_HOME:?请设置 ANDROID_NDK_HOME}"
JNI=android/app/src/main/jniLibs
# 1) Rust → libapp_client.so（Profile android = release + strip；Bevy 很大，内存紧张时用 -j2）
cargo ndk -t arm64-v8a -o "$JNI" build --profile android -p app_client --lib -j"${JOBS:-3}"
# 2) libc++_shared.so（oboe/cpal 的 C++ 依赖）
cp "$ANDROID_NDK_HOME/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so" "$JNI/arm64-v8a/"
# 3) Gradle 打包 + 签名
cd android
[ -f local.properties ] || echo "sdk.dir=${ANDROID_HOME:?请设置 ANDROID_HOME}" > local.properties
./gradlew --no-daemon assembleRelease
ls -la app/build/outputs/apk/release/
