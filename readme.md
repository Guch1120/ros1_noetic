# 構成  
treeで表示させたものをブロック表示させる。  

# 実行方法  


# エラーとその解決法まとめ

## gitにsshkeyを保存するとき、"ssh-rsa ......="のとこまでコピペする。
ssh-rsaの後には半角スペースがあるがそのままで問題ない。
"ssh-rsa"の部分が抜けていたり、"="が抜けているとエラーとなる。

## 1. エラー: `roscore`実行時のエラー

### 実行コマンド
```bash
roscore
```

### エラー名
```
Error: unable to open display
```

### 解決するために行ったコマンド
```bash
xhost +local:dockeruser
```

### エラー解説と解決方法の解説
- **エラー解説**: DockerコンテナからX11を利用してGUIアプリケーションを実行する場合、ホストのディスプレイを利用する設定が必要です。このエラーは、`roscore`を実行しようとした際に、X11ディスプレイの設定が適切でなかったために発生しました。
- **解決方法**: `xhost +local:dockeruser`を実行して、ローカルユーザーがX11ディスプレイにアクセスできるように許可します。その後、コンテナ内でGUIアプリケーションを実行できます。

---

## 2. エラー: `roscore`実行時の`ROS_MASTER_URI not set`

### 実行コマンド
```bash
roscore
```

### エラー名
```
ROS_MASTER_URI not set
```

### 解決するために行ったコマンド
```bash
export ROS_MASTER_URI=http://localhost:11311
```

### エラー解説と解決方法の解説
- **エラー解説**: `roscore`が起動しない理由として、ROSのマスターサーバーのURIが設定されていなかったことが考えられます。`ROS_MASTER_URI`は、ROSのマスターサーバーの場所を指し示す環境変数です。
- **解決方法**: `export ROS_MASTER_URI=http://localhost:11311`を実行して、マスターサーバーのURIを設定しました。これにより、`roscore`を正しく起動できるようになります。

---

## 3. エラー: `rosdep`でパッケージ解決できないエラー

### 実行コマンド
```bash
rosdep install tf
```

### エラー名
```
ERROR: failed to resolve package [tf] at index [index]
```

### 解決するために行ったコマンド
```bash
rosdep update
```

### エラー解説と解決方法の解説
- **エラー解説**: `rosdep`が`tf`パッケージの依存関係を解決できなかった理由として、`rosdep`のキャッシュが古くなっていたため、パッケージが正しく解決できませんでした。
- **解決方法**: `rosdep update`を実行することで、`rosdep`のキャッシュを更新し、依存関係を再度解決できるようにしました。

---

## 4. エラー: `roscore`コマンドが見つからないエラー

### 実行コマンド
```bash
roscore
```

### エラー名
```
/entrypoint.sh: line 2: roscore: command not found
```

### 解決するために行ったコマンド
```dockerfile
RUN apt-get update && apt-get install -y ros-noetic-ros-core
```

### エラー解説と解決方法の解説
- **エラー解説**: `roscore`コマンドが見つからないというエラーが発生しました。これは、`roscore`がインストールされていないためです。
- **解決方法**: `ros-noetic-ros-core`をインストールすることで、`roscore`を使用できるようにしました。

---

# 初心者向け解説

## X11ディスプレイの設定について
- **X11ディスプレイ**: X11は、LinuxなどのUnix系オペレーティングシステムで、グラフィカルユーザーインターフェイス（GUI）を提供するためのシステムです。Dockerコンテナ内でGUIを実行するには、ホストのX11ディスプレイにアクセスする必要があります。このため、コンテナがホストのX11サーバーと通信できるように設定を行います。
- **xhostコマンド**: `xhost`コマンドは、X11サーバーのアクセス許可リストを管理するためのツールです。`xhost +local:dockeruser`は、ローカルユーザー（この場合は`dockeruser`）がX11サーバーにアクセスできるようにするコマンドです。

## ROS_MASTER_URIについて
- **ROS_MASTER_URI**: `ROS_MASTER_URI`は、ROSネットワーク内でマスターサーバーのURIを指定する環境変数です。`roscore`を実行する際には、ROSマスターサーバーの情報を指定する必要があります。通常、`localhost`（自分自身のマシン）を指定します。
- **`export ROS_MASTER_URI=http://localhost:11311`**: このコマンドは、ROSマスターサーバーのURIを設定するために使います。`11311`はROSのデフォルトポートです。

---

## Dockerfile・entrypoint.shについて

### Dockerfileについて
- **Dockerfile**は、コンテナを作成するための指示を記述するファイルです。`FROM`でベースとなるイメージを指定し、必要なソフトウェアを`RUN`コマンドでインストールします。最後に`ENTRYPOINT`でコンテナ起動時に実行するスクリプトやコマンドを指定します。

### entrypoint.shについて
- **entrypoint.sh**は、コンテナが起動する際に実行されるシェルスクリプトです。ここでは、ROSの環境を設定するために`source /opt/ros/noetic/setup.bash`を実行し、`roscore`をバックグラウンドで実行しています。その後、ユーザーが実行したコマンドを`exec "$@"`で処理します。

---

## RUN useraddコマンドの解説

### 実行コマンド
```dockerfile
RUN useradd -m -s /bin/bash dockeruser && echo "dockeruser:docker" | chpasswd && adduser dockeruser sudo
```

### 各引数の意味
- `useradd`: 新しいユーザーを作成するコマンド。
  - `-m`: ユーザーのホームディレクトリを作成します。
  - `-s /bin/bash`: ユーザーのシェルを`/bin/bash`に設定します。
- `echo "dockeruser:docker" | chpasswd`: ユーザー`dockeruser`のパスワードを`docker`に設定します。
- `adduser dockeruser sudo`: ユーザー`dockeruser`を`sudo`グループに追加し、管理者権限を付与します。

