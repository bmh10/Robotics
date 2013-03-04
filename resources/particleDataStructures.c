// Some suitable data structures for particle filter
// Andrew Davison 2008-2012

const int NUMBER_OF_PARTICLES = 100;
const int NUMBER_OF_WALLS = 8;

// Arrays for storing information about particles
float xArray[NUMBER_OF_PARTICLES];
float yArray[NUMBER_OF_PARTICLES];
float thetaArray[NUMBER_OF_PARTICLES];
float weightArray[NUMBER_OF_PARTICLES];
float cumulativeWeightArray[NUMBER_OF_PARTICLES];



// Definitions of walls
// a: O to A
// b: A to B
// c: C to D
// d: D to E
// e: E to F
// f: F to G
// g: G to H
// h: H to O
//                                      a    b    c    d    e    f    g    h
float wallAxArray[NUMBER_OF_WALLS] = {  0,   0,  84,  84, 168, 168, 210, 210};
float wallAyArray[NUMBER_OF_WALLS] = {  0, 168, 126, 210, 210,  84,  84,   0};
float wallBxArray[NUMBER_OF_WALLS] = {  0,  84,  84, 168, 168, 210, 210,   0};
float wallByArray[NUMBER_OF_WALLS] = {168, 168, 210, 210,  84,  84,   0,   0};


// Number of cm per pixel in display
const float DISPLAY_SCALE = 3.0;

void drawMap()
{
	// Display the map
	for (int j = 0; j < NUMBER_OF_WALLS; j++) {
		nxtDrawLine((int)(wallAxArray[j]/DISPLAY_SCALE), (int)(wallAyArray[j]/DISPLAY_SCALE),
			(int)(wallBxArray[j]/DISPLAY_SCALE), (int)(wallByArray[j]/DISPLAY_SCALE));
	}
}

void drawParticles()
{
	// Draw the particle set
	for (int i = 0; i < NUMBER_OF_PARTICLES; i++) {
		nxtSetPixel((int)(xArray[i]/DISPLAY_SCALE), (int)(yArray[i]/DISPLAY_SCALE));
	}

}

task main()
{
	eraseDisplay();

	drawMap();

	drawParticles();

	wait1Msec(5000);

}
