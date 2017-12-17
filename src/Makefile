CC = g++
CCSW = -O3 -Wno-deprecated-declarations
PLATFORM = `uname`

lander: lander_dynamics.o lander_graphics.o miscellaneous_functions.o orbiting_object.o kepler_solver.o model_obj.o
	@if [ ${PLATFORM} = "Linux" ]; \
	then $(CC) -o lander lander_dynamics.o lander_graphics.o miscellaneous_functions.o orbiting_object.o kepler_solver.o model_obj.o ${CCSW} -lGL -lGLU -lglut -lSOIL -lIrrKlang -pthread; \
	echo Linking for Linux; \
	elif [ ${PLATFORM} = "Darwin" ]; \
	then $(CC) -o lander lander_dynamics.o lander_graphics.o miscellaneous_functions.o orbiting_object.o kepler_solver.o model_obj.o ${CCSW} -lSOIL -framework GLUT -framework OpenGL -framework CoreFoundation; \
	echo Linking for Mac OS X; \
	else $(CC) -o lander lander_dynamics.o lander_graphics.o miscellaneous_functions.o orbiting_object.o kepler_solver.o model_obj.o ${CCSW} -lglut32 -lglu32 -lopengl32 -lSOIL; \
	echo Linking for Cygwin; \
	fi

lander_graphics.o miscellaneous_functions.o orbiting_object.o kepler_solver.o model_obj.o lander_dynamics.o: define_constants.h global_1.h global_2.h kepler_solver.h lander_dynamics.h lander_graphics.h model_obj.h orbiting_object.h other_data_types.h vector3d.h

.cpp.o:
	$(CC) ${CCSW} -c $<

clean:
	echo cleaning up; /bin/rm -f core *.o lander

all:	lander

