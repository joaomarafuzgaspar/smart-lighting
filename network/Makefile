# Makefile
BINDIR := bin

CC := g++ -std=c++17 -pedantic -Wall -ggdb

SRC := $(wildcard src/*.cpp)
OBJ := $(patsubst src/%.cpp, $(BINDIR)/%.o, $(SRC))
INC := $(wildcard include/*.h)

all: bin client server build

client: $(BINDIR)/main_client.o | $(INC)
	$(CC) $< -pthread -o $(BINDIR)/client -lboost_program_options

server: $(BINDIR)/main_server.o | $(INC)
	$(CC) $< -pthread -o $(BINDIR)/server -lboost_program_options

build: $(OBJ) | $(INC)

$(BINDIR)/%.o: src/%.cpp | $(INC)
	$(CC) -I include -c $< -o $@ -fPIC

.PHONY: bin
bin:
	mkdir -p bin

.PHONY: clean
clean:
	rm -f bin/* lib/*