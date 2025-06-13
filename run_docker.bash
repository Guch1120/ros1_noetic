#!/bin/bash

# 起動中のdocker-composeサービスを取得
# docker-compose.yml が存在しない場合やDockerデーモンが停止している場合のエラー出力を抑制
RUNNING_SERVICES=$(docker-compose ps --services --filter "status=running" 2>/dev/null)

if [ -n "$RUNNING_SERVICES" ]; then
    # 起動中のサービスが1つ以上あれば、最初のサービスに入る
    # (複数のサービスが起動している場合、リストの最初のものが選択される)
    FIRST_SERVICE=$(echo "$RUNNING_SERVICES" | head -n 1)
    echo "コンテナ ($FIRST_SERVICE) は既に起動しています。コンテナ ($FIRST_SERVICE) に入ります..."
    # docker-compose exec を実行し、terminatorを起動する
    docker-compose exec "$FIRST_SERVICE" terminator
else
    echo "起動中のコンテナはありません。コンテナを起動します..."
    # Ctrl+Cが押されたときに実行される関数
    cleanup() {
        echo "Ctrl+Cが押されました。コンテナを停止します..."
        docker-compose stop
        exit 0 # trapの後、スクリプトを正常終了させる
    }

    # SIGINT (Ctrl+C) シグナルを捕捉し、cleanup関数を呼び出す
    trap cleanup INT

    # docker-compose up をフォアグラウンドで実行
    docker-compose up
    # docker-compose up が Ctrl+C 以外で終了した場合 (例: コンテナが正常終了)、trap は実行されない
fi