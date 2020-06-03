CXXFLAGS = -Ofast -I.

build: main.exe

main.exe: main.o
	$(CXX) -o $@ $^

main.o: denoiser.hpp envDetector.hpp kalmanFilter.hpp

test: main.exe testInput.txt
	./main.exe < testInput.txt > testOutput.txt

clean:
	rm main.exe main.o

.PHONY: build test clean
