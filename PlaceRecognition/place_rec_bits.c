#pragma config(Sensor, S4,     sonarSensor,         sensorSONAR)

#define PI 3.14159265358979
#define SCREEN_MAX_X 99
#define SCREEN_MAX_Y 63
#define DISPLAY_SCALE 0.3

#define THRESHOLD 1000
#define SONAR_MAX 255
#define BUCKET_SIZE 5

/// Number of bins (measurements) considered to characterize one location
#define NO_BINS 72
#define NO_BUCKETS SONAR_MAX/BUCKET_SIZE


/// Number of locations we want to learn in the environment
#define NO_LOCS 3

/// The size of the data that is going to be written to files
int nFileSize	= NO_BINS * 2;

/// A table containing the names of the files we are going to read / write
string file_names[NO_LOCS];


/// The location signature structure: stores the signature characterizing one location
typedef struct
{
	short sig[NO_BINS];
} loc_sig;

int depthHistogram_obs[NO_BUCKETS];
int depthHistogram_read[NO_BUCKETS];


// --------------------- File management functions ---------------
/** This function fills the file_names global variable with names
    like loc_%%.dat where %% are 2 digits (00, 01, 02...) indicating
    the location number. Up to NO_LOCS (i.e, the number of locations
    in the environment) are going to be assigned a file name */
void form_file_names()
{
  for ( short n=0; n<NO_LOCS; n++ )
    {
      StringFormat(file_names[n], "loc_%02d.dat", n);
    }
}



/** Given that some location files have been stored, and given
    that at most NO_LOCS are going to be learned, what is the
    next location index that is expected to be written? For instance,
    if 2 locations out of 3 have already been recorded in files
    loc_00.dat and loc_01.dat, then, the next "free" index for
    naming a file would be 2, and the corresponding file would be
    loc_02.dat */
short find_next_loc_index()
{
  short n = 0;
  while ( n<NO_LOCS )
    {
      TFileIOResult nIoResult;
      TFileHandle hFileHandle;
      int read_size;

      OpenRead(hFileHandle, nIoResult, file_names[n], read_size);
      bool exists = (nIoResult == 0);
      Close(hFileHandle, nIoResult);
      if ( !exists )
	{
	  Delete(file_names[n], nIoResult);
	  break;
	}
      else
	n++;
    }

  return n;
}


/// Delete all loc_%%.dat files
void delete_loc_files()
{
  for ( short n=0; n<NO_LOCS; n++ )
    {
      TFileIOResult nIoResult;
      TFileHandle hFileHandle;
      int read_size;

      OpenRead(hFileHandle, nIoResult, file_names[n], read_size);
      bool exists = (nIoResult == 0);
      Close(hFileHandle, nIoResult);
      if ( exists )
	Delete(file_names[n], nIoResult);
    }
    writeDebugStreamLine("DELETING LOC FILES");
}



/** Writes the signature ls to the file whose named can be
    identified using index (e.g, if index is 1, then ls is going
    to be written in file loc_01.dat). If file already exists,
    it should be replaced (FOR SECURITY, IT IS BETTER THAT FILE
    DID NOT ALREADY EXIST). */
void write_signature_to_file(loc_sig& ls, short index)
{
  ASSERT(index >= 0 && index < NO_LOCS);
  TFileIOResult nIoResult;
  TFileHandle hFileHandle;

  string sFileName = file_names[index];
  Delete(sFileName, nIoResult);

  OpenWrite(hFileHandle, nIoResult, sFileName, nFileSize);
  for ( short i=0; i<NO_BINS; ++i )
    WriteShort(hFileHandle, nIoResult, ls.sig[i]);

  Close(hFileHandle, nIoResult);
}



/** Read file corresponding to index (e.g., if index is 1,
    read file will be loc_01.dat), and store its signature
    into ls. ASSUMES FILE EXISTS! */
void read_signature_from_file(loc_sig& ls, short index)
{
  ASSERT(index >= 0 && index < NO_LOCS);
  TFileIOResult nIoResult;
  TFileHandle hFileHandle;
  int read_size;

  string sFileName = file_names[index];
  OpenRead(hFileHandle, nIoResult, sFileName, read_size);
  for ( short i=0; i<NO_BINS; ++i )
    ReadShort(hFileHandle, nIoResult, ls.sig[i]);

  Close(hFileHandle, nIoResult);
}

void rotateSonar(float angle)
{
    int i = (angle < 0) ? -1 : 1;
    motor[motorA] = i*25;
    wait1Msec(i*angle*4);
    motor[motorA] = 0;
}


void characterize_location(loc_sig ls)
{
	float ang = 360/NO_BINS+2.90;
	// Spin sonar and take readings
  for (int i=0; i < NO_BINS; i++)
  {
    rotateSonar(ang);
    // Save dist in loc_sig instance ls
    ls.sig[i] = (short) SensorValue[sonarSensor];
  }
  // Unspin sonar back to start position
  rotateSonar(-360);
}

float degToRad(float deg)
{
  return (deg*PI)/180.0;
}

void drawLines(loc_sig ls)
{
	short s;
	float ang = 360/NO_BINS;

	for (int i=0; i < NO_BINS; i++) {
		s = (int)ls.sig[i];
		writeDebugStreamLine("%d", s);
	  //nxtDrawLine(SCREEN_MAX_X/2, SCREEN_MAX_Y/2, s*cos(i*ang), s*sin(i*ang));
  }
}

/** This function simply characterizes the current location,
    and stores the obtained signature into the next available
    file index (e.g., 2 if loc_00.dat and loc_01.dat already
    exist). WARNING: AT MOST NO_LOCS CAN BE LEARNED. */
void learn_location()
{
	loc_sig ls;
	characterize_location(ls);
	short index = find_next_loc_index();
	writeDebugStreamLine("WRITING LOCATION IDX: %d", index);
	write_signature_to_file(ls, index);

	//Display lines on screen
	//drawLines(ls);
}

void makeDepthHistogram(int n, loc_sig ls)
{
	int i;
writeDebugStreamLine("---START---");

  if (n==0) {
	  for (i=0; i < NO_BUCKETS; i++) {
		  	depthHistogram_obs[i] = 0;
    }
    for (int i=0; i < NO_BINS; i++) {
  	  int idx = (ls.sig[i]/BUCKET_SIZE);
  	  if (idx == NO_BUCKETS) idx--;
  	  writeDebugStreamLine("IDX: %d %d", idx, ls.sig[i]);
      depthHistogram_obs[idx]++;
    }
    for (short i=0; i < NO_BUCKETS; i++) {
      int s = depthHistogram_obs[i];
      writeDebugStreamLine("%d", s);
    }
  }
  else
  {
  	for (i=0; i < NO_BUCKETS; i++) {
		  	depthHistogram_read[i] = 0;
    }
    for (int i=0; i < NO_BINS; i++) {
  	  int idx = (ls.sig[i]/BUCKET_SIZE);
  	  if (idx == NO_BUCKETS) idx--;
  	  writeDebugStreamLine("IDX: %d", idx);
      depthHistogram_read[idx]++;
    }
    for (short i=0; i < NO_BUCKETS; i++) {
      short s = depthHistogram_read[i];
      writeDebugStreamLine("%d", (int) s);
    }
  }
}

/** This function tries to recognize the current location.
    1. Characterize current location
    2. For every learned locations
    2.1. Read signature of learned location from file
    2.2. Compare signature to signature coming from actual characterization
    3. Retain the learned location whose minimum distance with
    actual characterization is the smallest.
    4. Display the index of the recognized location on the screen */
void recognize_location()
{
  loc_sig ls_obs, ls_read;
  characterize_location(ls_obs);
  float bestScore = -1.0;
  float bestScoreAngle = -1.0;
  short matchIndex = -1;
  float a;
  float angle = 0.0;

  makeDepthHistogram(0, ls_obs);


  for ( short n=0; n < NO_LOCS; n++ )
  {
    read_signature_from_file(ls_read, n);
    makeDepthHistogram(1, ls_read);

    a = 0.0;
    // COMPARE ls_read with ls_obs and find the best match
    for (int i=0; i < NO_BUCKETS; i++)
    {
  	  a += pow(depthHistogram_obs[i] - depthHistogram_read[i], 2);
    }

    writeDebugStreamLine("FILE %d: a=%f", (int) n,  a);

    if (a < bestScore || bestScore == -1.0) // && a < THRESHOLD)
    {
    	bestScore = a;
    	matchIndex = n;
    }
  }

  // Location match found, now find robots orientation relative to stored signature
  read_signature_from_file(ls_read, matchIndex);
  for (int i=0; i < NO_BINS; i++)
  {
  	// Starting at each different bin in the signature calculate a score and then
    // use the starting position with the highest score to calculate the robots orientation.
  	a = 0.0;
    for (int j=0; j < NO_BINS; j++)
    {
  	   a += (pow((ls_obs.sig[(i+j)%NO_BINS] - ls_read.sig[(i+j)%NO_BINS]), 5));
    }
     writeDebugStreamLine("ANGLE SCORE: %d %f", i, a);

    if (a < bestScoreAngle || bestScoreAngle == -1.0) // && a < THRESHOLD)
    {
    	bestScoreAngle = a;
    	angle = i*(360/NO_BINS); // Angle equal to index we are at * angle per bin
    	 writeDebugStreamLine("ANGLE SET: %f %f", angle, bestScoreAngle);
    }
  }

  // Display output
  nxtDisplayStringAt(10, 30, "MATCH: %d", (int) matchIndex);
  nxtDisplayStringAt(10, 20, "SCORE: %f", bestScore);
  nxtDisplayStringAt(10, 10, "ANGLE: %f", angle);
}



/** First thing to do, is to form file names.
    Prior to starting learning the locations, it
    could be good to delete files from previous
    learning by calling delete_loc_files(). This
    could also be done manually using RobotC. Then,
    either learn a location, until all the locations
    are learned, or try to recognize one of them, if
    locations have already been learned. Last infinite
    while loop is just for the display to hold on. */
task main()
{
	writeDebugStreamLine("---------------------------------");
	nMotorPIDSpeedCtrl[motorA] = mtrSpeedReg;
  form_file_names();


  //delete_loc_files();

  //learn_location();

  recognize_location();

  while ( true )
  {
    wait10Msec(100);
  }

  return;
}
