CXX = g++
CXXFLAGS = -D _DEBUG -D _SHOW_STACK_TRACE -ggdb3 -std=c++17 -O0 -Wall -g
files = Rx.cpp Tx.cpp RxEth.cpp TxEth.cpp main.cpp 
headers = TxRx.h
objects = $(files:.cpp=.o)

buildDir = build
srcDir = src

.PHONY: all clean prepare
all: prepare main

$(addprefix $(buildDir)/, $(objects)): $(buildDir)/%.o: $(srcDir)/%.cpp
		@echo [CC] $< -o $@
		@$(CXX) $(CXXFLAGS) -Iinclude -c $(realpath $<) -o $@

main: $(addprefix $(buildDir)/, $(objects))
		@echo [CC] $^ -o $@
		@$(CXX) $(CXXFLAGS) $^ -o $(addprefix $(buildDir)/, $@)

clean:
		rm -f build/*.o all
		rm build/cte

prepare:
		@mkdir -p $(buildDir)