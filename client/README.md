# client

To run the client you need to install the following:
- rosbridge
- Node.js
- npm (package manager for Node.js)

## Install rosbridge server
```sh
sudo apt-get install ros-noetic-rosbridge-server
```

## Add the following lines to the end of the launch file(s)
```xml
<include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch"> 
    <arg name="port" value="8080"/>
</include>
```

## Install node & npm

First add a repository containing the latest version of node (Nodesource.com in this example)

```sh
curl -sSL https://deb.nodesource.com/setup_16.x | sudo bash -
```

Then you can install node and npm with the following command:

```sh
sudo apt install -y nodejs
```

To check if they're installed just run:

```sh
npm -v
node -v
```

## Project Setup

```sh
npm install
```

### Compile and Hot-Reload for Development

```sh
npm run dev
```

### Compile and Minify for Production

```sh
npm run build
```
