# Indicates compilator to use
CC      = g++

# Specifies compilator options
CFLAGS  = -O3 -Wall -fPIC `pkg-config --cflags opencv` -D LINUX
LDFLAGS = 
LDLIBS  = `pkg-config --libs opencv`

# Files extensions .cpp, .o
SUFFIXES = .cpp .o 
.SUFFIXES: $(SUFFIXES) .

# Name of the main program
PROG  = surf

# Name of the library
LIB = lib$(PROG)
DLIB = $(LIB).so
SLIB = $(LIB).a
LIBS = $(SLIB) $(DLIB)

# Object files .o necessary to build the main program
OBJS  = fasthessian.o integral.o main.o surf.o utils.o ipoint.o

# Object files .o necessary to build the library
LIBOBJS  = fasthessian.o integral.o surf.o utils.o ipoint.o
 
all: $(PROG) $(LIBS)

# Compilation and link
$(PROG): $(OBJS)
	$(CC) $(LDFLAGS) -o $(PROG) $(OBJS) $(LDLIBS)

$(DLIB): $(LIBOBJS)
	$(CC) $(LIBOBJS) -o $(DLIB) -ldl -shared -fPIC

$(SLIB): $(LIBOBJS)
	ar rcs $(SLIB) $(LIBOBJS)

%.o: $(addprefix ./src/, %.cpp)
	$(CC)  $(CFLAGS) -c $< -o $@

clean:
	-rm -f $(PROG)
	-rm -f $(LIBS)
	-rm -f *.o
