all: scope

scope: scope.o
	gcc -o scope scope.o

scope.o: scope.c header.h
	gcc -c scope.c

clean:
	rm -f *.o
