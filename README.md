# Color-Filtered Robot Navigation System

A comprehensive 2D robot navigation system with advanced color filtering capabilities for maze processing and real-time simulation.

## 🚀 Features

### Advanced Color Filtering
- **Smart obstacle detection** that excludes robot and goal colors
- **HSV color space filtering** for accurate color-based object separation
- **Automatic maze processing** from image files with color analysis
- **Visual debugging tools** with color mask overlays

### Real-time Navigation
- **WebSocket-based communication** between server and client
- **Interactive HTML simulator** with canvas-based visualization
- **RESTful API** for robot control and goal management
- **Collision detection** and goal reaching verification

### Intelligent Path Planning
- **Obstacle avoidance** with configurable safety margins
- **Multiple navigation modes** (absolute positioning, relative movement)
- **Performance tracking** and success rate monitoring

## 📋 Requirements

### Python Dependencies

### System Requirements
- Python 3.7+
- Modern web browser with WebSocket support
- Camera or image files for maze processing

## 🛠️ Installation

1. **Clone the repository**

2. **Install dependencies**

3. **Start the server**

4. **Open the simulator**
   - Open `simulator.html` in your web browser
   - The simulator will connect to `ws://localhost:8080`

5. **Run the navigation client**

## 🎮 Usage

### Server Endpoints

The Flask server (`server.py`) provides these REST API endpoints:

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/move` | Move robot to absolute position |
| POST | `/move_rel` | Move robot relative to current position |
| POST | `/goal` | Set navigation goal |
| GET | `/goal/status` | Check if goal is reached |
| POST | `/obstacles/random` | Generate random obstacles |
| POST | `/obstacles/positions` | Set custom obstacle positions |
| POST | `/reset` | Reset simulation state |
| GET | `/status` | Get system status |

### Color-Filtered Navigation

The main client (`final1.py`) offers an interactive menu system:

1. **Load Maze** - Process image files with color filtering
2. **Test Navigation** - Automated pathfinding tests
3. **Maze Status** - View processing results and statistics
4. **Custom Navigation** - Manual goal setting
5. **Performance Stats** - Success rates and metrics

### HTML Simulator

The web-based simulator (`simulator.html`) provides:
- Real-time robot visualization
- Interactive obstacle placement
- Goal setting via mouse clicks
- WebSocket communication with server

## 🎨 Color Filtering System

### Supported Colors
- **Red Robot**: HSV ranges for circular robot detection
- **Green Goal**: HSV ranges for square goal detection
- **Gray Obstacles**: Automatic detection of maze walls and barriers

### Configuration

## 🔧 Configuration

### Canvas Settings
- CANVAS_WIDTH = 650
- CANVAS_HEIGHT = 600
- ROBOT_RADIUS = 18
- GOAL_RADIUS = 15
- OBSTACLE_SIZE = 25

### Navigation Parameters
- GOAL_TOLERANCE = 15
-  MOVEMENT_DELAY = 0.1
- SAFETY_MARGIN = 40
- 
## 🏗️ Architecture

### Components
1. **Flask Server** (`server.py`) - RESTful API and WebSocket handling
2. **Navigation Client** (`final1.py`) - Color filtering and pathfinding logic
3. **HTML Simulator** (`simulator.html`) - Web-based visualization interface

### Communication Flow

## 📈 Advanced Features

### Maze Processing Pipeline
1. **Image loading** with automatic resizing
2. **Color space conversion** to HSV
3. **Exclusion mask creation** for robot/goal areas
4. **Contour detection** with area filtering
5. **Obstacle extraction** with size validation

### Navigation Intelligence
- **Multi-threshold processing** for optimal obstacle detection
- **Safety margin enforcement** around obstacles
- **Goal reaching verification** with tolerance zones
- **Collision tracking** and avoidance

## 🚀 Quick Start

1. **Start the server:**

2. **Open simulator in browser:**

4. **Follow the interactive menu** to test different features!

## 📝 Example Usage

### API Usage Examples
- Set a goal position
- curl -X POST http://localhost:5001/goal -H "Content-Type: application/json" -d '{"x": 300, "y": 200}'

- Move robot to position
- curl -X POST http://localhost:5001/move -H "Content-Type: application/json" -d '{"x": 100, "y": 150}'

- Check goal status
- curl http://localhost:5001/goal/status


## 📄 Files Structure

├── server.py # Flask server with WebSocket support
├── final1.py # Main navigation client with color filtering
├── simulator.html # Web-based visualization interface
└── README.md # This file

