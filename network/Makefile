# Makefile
BINDIR := bin
LIBDIR := lib

CC := g++ -std=c++17 -pedantic -Wall -ggdb

VPATH = main:src

SRC := $(wildcard src/*.cpp)
OBJ := $(patsubst %.cpp, $(BINDIR)/%.o, $(notdir $(SRC)))
INC := $(wildcard src/*.h)

all: bin lib client server build

client: $(BINDIR)/main_client.o lib | $(INC)
	$(CC) $< -pthread -o $(BINDIR)/client -Llib -lscdtr

server: $(BINDIR)/main_server.o lib | $(INC)
	$(CC) $< -pthread -o $(BINDIR)/server -Llib -lscdtr

lib: $(LIBDIR)/libscdtr.a

$(LIBDIR)/libscdtr.a: $(OBJ) | $(INC)
	mkdir -p lib
	ar rcs $@ $(OBJ)

build: $(OBJ) | $(INC)

$(BINDIR)/%.o: %.cpp | $(INC)
	$(CC) -I src -c $< -o $@ -fPIC

.PHONY: bin
bin:
	mkdir -p bin

.PHONY: clean
clean:
	rm -f bin/* lib/*