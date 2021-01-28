/*****************************/
/*                           */
/* Date: September 2017      */
/* Author: S. Biondi         */
/*****************************/

/* A simple server in the internet domain using TCP
   The port number is passed as an argument 
   This version runs forever;
   it handles one external connection at a time.
*/

#include <iostream>
#include <string>
#include <vector>
#include <time.h>
#include "SOCServerInterface.hh"
#include "SOCSimuServerInterface.hh"
#include "TDAQServerBase.hh"

using namespace std;

// main function calling the thread for slow connection:
// it will call the fast connection 
int main( int argc, char *argv[] ){

  cout << argv[0] << " node_ip_address portNumber [verbose] [simulation] [blockSize]" << endl; 
  vector<string> arguments;
  for ( int i = 0; i< argc; i++ ){
    arguments.push_back(string(argv[i]));
  }
  unsigned int verbose = 0;
  if( argc>2 ) verbose=atoi(arguments[3].c_str());
  cout << "argc=" << argc<<endl << endl; 
  cout << "argv[0]" << argv[0] << endl; 
  cout << "argv[1]" << argv[1] << endl; 
  cout << "argv[2]" << argv[2] << endl; 
  cout << "argv[3]" << argv[3] << endl; 

  DAQServerInterface * p_servInt = NULL;
  if( arguments.size()<5 || atoi(arguments[4].c_str())==0 ){
    cout << "RUNNING WITH DATA FROM FPGA..." << endl << endl; 
    // the HPS-FPGA Handler that knows the TDAQ transitions
    p_servInt = new SOCServerInterface(verbose);
  } else {
    cout << "RUNNING WITH SIMULATED DATA " << endl << endl; 
    // the Server interface that produces SOC simulated data
    p_servInt = new SOCSimuServerInterface(verbose);
  }
  
  // main server object
  TDAQServerBase  tdaqServer(arguments, p_servInt);

  // start thread for sharing slow info: configure and monitoring
  cout << "Connecting the slow thread..." << endl << endl; 
  tdaqServer.runServer(); //actual running of the server; no return till end...

  return 0;

}
