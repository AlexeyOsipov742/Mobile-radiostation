CXX = g++

# Важно: используем std::thread -> нужен -pthread
CXXFLAGS = -O2 -g -Wall -std=c++17 -pthread

files = Rx.cpp Tx.cpp RxEth.cpp TxEth.cpp main.cpp audioRxEth_PI.cpp audioTxEth_PI.cpp
objects = $(files:.cpp=.o)

buildDir = build
srcDir = src

# ALSA больше не нужна (аудио через MCP3201/MCP4822)
LIBS = -lwiringPi -pthread

.PHONY: all clean prepare
all: prepare main

$(addprefix $(buildDir)/, $(objects)): $(buildDir)/%.o: $(srcDir)/%.cpp
		@echo [CC] $< -o $@
		@$(CXX) $(CXXFLAGS) -Iinclude -c $(realpath $<) -o $@

main: $(addprefix $(buildDir)/, $(objects))
		@echo [CC] $^ -o $@
		@$(CXX) $(CXXFLAGS) $^ -o $(addprefix $(buildDir)/, $@) $(LIBS)

clean:
		rm -f build/*.o all
		rm -f build/cte

prepare:
		@mkdir -p $(buildDir)
debug: 
		