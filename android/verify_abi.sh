#!/usr/bin/env bash
# 校验 APK 内 Java 层 GameActivity 与 libapp_client.so 里静态链入的 C 层 GameActivity 的
# JNI 注册签名一致（STRATO-32 闪退 SIGABRT 的根因：两端版本错配）。
# 用法：android/verify_abi.sh [apk]   需要 ANDROID_HOME（用 build-tools 的 dexdump）和 unzip/strings
set -euo pipefail
cd "$(dirname "$0")"
APK="${1:-app/build/outputs/apk/release/app-release.apk}"
T="$(mktemp -d)"; trap 'rm -rf "$T"' EXIT
unzip -q -o "$APK" 'classes*.dex' 'lib/arm64-v8a/libapp_client.so' -d "$T"

# Native 端：RegisterNatives 表里的签名串
NATIVE="$(strings -a "$T/lib/arm64-v8a/libapp_client.so" | grep -E '^\(JLandroid/view/MotionEvent;[A-Za-z;/]*\)Z$' | sort -u)"

# Java 端：dex 里 GameActivity.onTouchEventNative 的方法原型
DEXDUMP="$(ls "${ANDROID_HOME:?}"/build-tools/*/dexdump | tail -1)"
JAVA="$("$DEXDUMP" -d "$T"/classes*.dex 2>/dev/null \
  | awk '/Class descriptor.*Lcom\/google\/androidgamesdk\/GameActivity;/{c=1} c&&/name +: .onTouchEventNative./{n=1} n&&/type +:/{print; exit}' \
  | sed -E "s/.*type +: '(.*)'.*/\1/")"

echo "native: ${NATIVE:-<none>}"
echo "java  : ${JAVA:-<none>}"
if [ -z "$JAVA" ] || [ -z "$NATIVE" ]; then echo "ERROR: 无法提取签名" >&2; exit 2; fi
if [ "$JAVA" != "$NATIVE" ]; then
  echo "ERROR: GameActivity Java/native JNI 签名不一致 → 运行时 RegisterNatives 失败并 abort()。" >&2
  echo "       请让 android/app/build.gradle 的 games-activity 版本与 android-activity 匹配（见 docs/ANDROID_PORT.md）。" >&2
  exit 1
fi
echo "OK: Java/native 签名一致"
