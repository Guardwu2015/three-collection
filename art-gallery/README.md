# gallery（数字展馆）

![cover.jpg](./cover.jpg)

#### 介绍
本项目中使用的技术栈为`three.js`，使用`blender`进行建模，最后烘焙渲染场景贴图，导出`glb`地图格式在Web端渲染。  
此项目仅为数字展馆概念的demo项目，如有不完善的地方还请多多包涵，有任何问题都可以提issue。

#### 对于此开源项目的个人想法： 
受threejs官网demo项目的灵感，在开发此项目时尝试了很多方案，也看到有很多类似的项目，但大多这类型的项目都是明码标价售卖的（格局太小了），反观老外的github上，有大量优秀的3D开源项目，这也正是别人技术发展如此迅速的原因。
因此本人还是觉得不要吝啬于贡献技术，多多开源项目，给到更多人灵感，相互提供帮助才能更好地推动行业发展。

> 开源不易，多多Star⭐⭐⭐  

#### 🎇feature：  
1. 高性能碰撞检测：  
因为这类项目对于物理引擎的应用场景并不多，经过不懈的技术方案调研后使用了一套不依赖于物理引擎的高性能的动态碰撞检测方案。比`three.js`官网的`Octree`方案性能还要好上几倍。
2. 画展交互：  
利用光线投射进行物体探测触发互动效果。
3. 位置音频：  
加入了位置音频，模拟现实中的听觉传播，使得场景中的音乐更具有空间感，提升浏览体验。

#### 运行
To setup a dev environment:
```text
# Clone the repository

# Install dependencies
npm i

# Run the local dev server
npm run dev
```
To serve a production build:
```text
# Install dependencies if not already done - 'npi i'

# Build for production
npm run build
```
