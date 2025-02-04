Physics Engine
A Physics Engine written in C using the SDL2 library to display simulations.

Features
Simulates basic physics principles.
Utilizes the SDL2 library for rendering.
Requirements
SDL2: Simple DirectMedia Layer 2.0
Installation
Clone the Repository

bash
Copy
Edit
git clone https://github.com/Mohamedb91/Physics-Engine.git
cd Physics-Engine
Install SDL2

On Ubuntu/Debian:

bash
Copy
Edit
sudo apt-get update
sudo apt-get install libsdl2-dev
On macOS (using Homebrew):

bash
Copy
Edit
brew install sdl2
On Windows:

Download the SDL2 development libraries from the SDL2 website and follow the installation instructions provided.

Compile the Program

bash
Copy
Edit
gcc -o physics_engine main.c Circle.c -lSDL2
Ensure that the SDL2 library is correctly linked during compilation.

Run the Program

bash
Copy
Edit
./physics_engine
On Windows, you might need to run physics_engine.exe instead.

Usage
Upon running the program, a window will display the physics simulation. Interact with the simulation as per the implemented controls.

Contributing
Contributions are welcome! Please fork the repository and submit a pull request with your changes.

License
This project is licensed.
