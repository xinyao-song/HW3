# Delete existing images
rm res.png

# Compile the program
g++ main.cpp rasterizer.cpp Triangle.cpp -I external/eigen-3.4.0 -o main.exe -std=c++17


# Run the program
./main.exe

# delete file
rm main.exe
