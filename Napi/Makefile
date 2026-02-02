CXX = g++
CXXFLAGS = -O2 -g -Wall -std=c++17 -pthread
DEBUG_FLAGS = -D _DEBUG -D _SHOW_STACK_TRACE -ggdb3 -std=c++17 -O0 -Wall -g -pthread
files = Rx.cpp Tx.cpp RxEth.cpp TxEth.cpp main.cpp audioRxEth_client.cpp audioTxEth_client.cpp command.cpp audio.cpp signal_handler.cpp gpio_control.cpp
headers = TxRx.h
objects = $(files:.cpp=.o)

buildDir = build
srcDir = src
LIBS = -lgpiod -lm

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
		
