#!/bin/bash

# Ctrl+Cが押されたときに実行される関数
cleanup() {
    echo "Ctrl+Cが押されました。コンテナを停止・削除します..."
    docker-compose stop
    exit 0
}

# SIGINT (Ctrl+C) シグナルを捕捉し、cleanup関数を呼び出す
trap cleanup INT

# docker-compose up をバックグラウンドで実行しない（フォアグラウンドで実行し、Ctrl+Cで直接終了できるようにする）
# docker-compose up のログはそのままターミナルに出力される
docker-compose up

# スクリプトが予期せず終了した場合のフォールバック (通常は上記のtrapで処理される)
cleanup