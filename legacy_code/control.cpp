#define RUN 
#define SOLVE_MAP
//#define TEST
#define DEBUG
//#define SUPERDEBUG

#include <Arduino.h>
#include "state_machines.h"
#include "robot.h"
#include "path_handler.h"


extern robot_t robot;

stack<char> path_taken, solved_path;
//vector <char> movements_made;

// Variáveis globais
bool END_MAP = false;
bool END_SOLVE = false;

bool cross=false;
int counter_align=0;
static unsigned long forwardStartTime = 0;

char node = ' ';

//edge detection variables
int p_START_BUTTON = 0;
int re_START_BUTTON = 0;




// Cycle time variables
unsigned int start_time = 0, end_time = 0, cycle_time = 0;

// Timers (exemplo)


// timerBlock timer_Exampler

void edge_detection()
{
    bool currentButtonState = digitalRead(START_BUTTON);

    //if ((millis() - lastDebounceTime) > debounceDelay) {
        if (!p_START_BUTTON && currentButtonState) {
            re_START_BUTTON = true;
        } 
        else {
            re_START_BUTTON = false;
        }
        p_START_BUTTON = currentButtonState;
  
    //}
	#ifdef DEBUG
	Serial.printf("-- Edges re_BUTTON=%d p_BUTTON=%d\n", re_START_BUTTON, p_START_BUTTON);
	#endif
}

void update_timers()
{
	// end_time = get_time();

	// if (start_time == 0)
	//     cycle_time = 0;
	// else
	//     cycle_time = end_time - start_time;

	// start_time = end_time;

	// Exemplo:
	// if (timer_State.on == 1)
	//     timer_State.time += cycle_time;

	// #ifdef DEBUG
	// printf("-- Timers timer_State.on=%d timer_State.time=%d\n", timer_State.on, timer_State.time);
	// #endif
}



void start_timer(timerBlock* t)
{
	t->on = 1;
	t->time = 0;
}

void stop_timer(timerBlock* t)
{
	t->on = 0;
	t->time = 0;
}






#ifdef RUN
void Main_FSM_Handler() 
{

switch (robot.currentStateMain) 
		{
			
			case IDLE_MAIN :
			
				#ifdef DEBUG
				printf("-- Current state main  = IDLE\n");
				#endif

				
				// if (digitalRead(START_BUTTON)) 
				// {
    					robot.currentStateMain = MAP;
        // }

				
				break;
			
			case MAP:
			
				#ifdef DEBUG
				printf("-- Current state main = MAP\n");
				#endif			
			
				
				if(END_MAP)
				{
					robot.currentStateMain = READY;
				}

			break;
			
			case READY :
			
				#ifdef DEBUG
				printf("-- Current state main = READY\n");
				#endif


				
				if(digitalRead(START_BUTTON))
				{
					robot.currentStateMain = SOLVE;
				}

        if(digitalRead(RESET_BUTTON))
        {
          robot.currentStateMain = MAP;
        }

			break;
			
			case SOLVE:
			#ifdef DEBUG
			printf("-- Current state main = SOLVE\r\n");
			#endif
		
		
			if (END_SOLVE) 
			{
				robot.currentStateMain = SOLVED; 
			}
		
			break;
		
		case SOLVED:
			#ifdef DEBUG
			printf("-- Current state main = SOLVED\n");
			#endif
		
			
			if (digitalRead(START_BUTTON)) 
			{
        robot.currentStateMain = SOLVE;
			}

      if(digitalRead(RESET_BUTTON))
      {
        robot.currentStateMain = IDLE_MAIN;
      }
			break;

		} 
        
        
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------     
// Update outputs
switch(robot.currentStateMain)
    {
    case IDLE_MAIN:
        //stop motors
        // clear up instructions and temporary stacks 
        break;

    case MAP:
        //stop motors again( I guess)
        break;

    case READY:
        //copy instructions to temporary stack; (for running again if needed)
        break;

    default:
        break;
    }
}


void Map_FSM_Handler()
{
    switch(robot.currentStateMap)
		{

			case IDLE_MAP:

				#ifdef DEBUG
				printf("-- Current state map = IDLE\n");
				#endif

				if(robot.currentStateMain == MAP)
        {
          robot.currentStateMap = FOLLOW_LINE_MAP;
        }

			break;

			case FOLLOW_LINE_MAP:

				#ifdef DEBUG
				Serial.print("-- Current state map = FOLLOW_LINE\n");
				#endif
        node = robot.IRLine.detectNode();
        if(node == robot.past_node){
          robot.node_count++;
        } else {
          robot.node_count = 0;
          robot.past_node = node;
        }
        if(robot.node_count < NODE_DETECTION){
          robot.currentStateMap = FOLLOW_LINE_MAP;
        }
        else{
          
          Serial.print("Stable node detected: ");
          Serial.println(node);
        
          if(node=='W') //OOOOO
          {
            robot.currentStateMap = U_TURN;
            robot.start_time_turn = millis();
          }
          
          if(node=='R' || node=='B' || node=='L') //OOXXX || XXXXX
          {
            robot.currentStateMap = SMALL_FORWARD;
            robot.start_time_turn = millis();
          }
        }

				

      break;


      case U_TURN:

        #ifdef DEBUG
        Serial.print("-- Current state map = U_TURN\n");
        #endif
        
        robot.currentStateMap = FOLLOW_LINE_MAP;
        robot.node_stack.push_back('U');

			break;


			case LEFT_TURN_MAP:

        #ifdef DEBUG
        Serial.print("-- Current state map = LEFT_TURN\n");
        #endif
        
        robot.currentStateMap = FOLLOW_LINE_MAP;
        robot.node_stack.push_back('L');

      break;

      case RIGHT_TURN_MAP:

        #ifdef DEBUG
        Serial.print("-- Current state map = RIGHT_TURN\n");
        #endif
        
        robot.currentStateMap = FOLLOW_LINE_MAP;
        robot.node_stack.push_back('R');

      break;


      case SMALL_FORWARD:
        
        #ifdef DEBUG
        Serial.print("-- Current state map = SMALL_FORWARD\n");
        #endif
        node = robot.IRLine.detectNode();
        if( robot.past_node == 'L' ) //OOXXX
        {
          robot.currentStateMap = LEFT_TURN_MAP;
          robot.start_time_turn = millis();
          break;
        }else if( robot.past_node == 'R' && (node == 'W' || node == 'R') ) //XXXOO
        {
          robot.currentStateMap = RIGHT_TURN_MAP;
          robot.start_time_turn = millis();
          break;
        }else if( robot.past_node == 'R' && node == 'N') //XXXOO but not stable yet
        {
          robot.currentStateMap = FOLLOW_LINE_MAP;
          robot.start_time_turn = millis();
          break;
        }
        else if( robot.past_node == 'B' && node == 'B') //XXXOO but not stable yet
        {
          robot.currentStateMap = END;
          break;
        }else if( robot.past_node == 'B' && node == 'W' ) //XXXOO but not stable yet
        {
          robot.currentStateMap = LEFT_TURN_MAP;
          break;
        }else if( robot.past_node == 'B' && node == 'N' ) //XXXOO but not stable yet
        {
          robot.currentStateMap = LEFT_TURN_MAP;
        }

			break;

      case FORWARD_MAP:

        #ifdef DEBUG
        Serial.print("-- Current state map = FORWARD\n");
        #endif

        if(1)
        {
          robot.currentStateMap = FOLLOW_LINE_MAP;
        }

      break;

      case END:

        #ifdef DEBUG
        Serial.print("-- Current state map = END\n");
        #endif
        node = robot.IRLine.detectNode();
        robot.solveNodeStack();
        robot.printNodeStack();
        if(node == robot.past_node){
          robot.node_count++;
        } else {
          robot.node_count = 0;
          robot.past_node = node;
        }
        if(robot.node_count < 5000){
          robot.currentStateMap = END;
        }else{
          if(node == 'N'){
            robot.currentStateMap = IDLE_MAP;
            robot.currentStateMain = SOLVE;
          }
        }
        if(0)
        {
          robot.currentStateMap = IDLE_MAP;
		    }

      break;
    }



    //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------     
 //update outputs

// Update outputs
switch(robot.currentStateMap)
    {
    case IDLE_MAP:
        
        break;

    case FOLLOW_LINE_MAP:
        //follow_line funtion

        robot.followLine(); 
        //Only real use of the PID
        break;

    case U_TURN:
        //U-turn function 
        robot.u_turn();
        //Just angular speed until -180 degrees (use encoders)
        break;

    case LEFT_TURN_MAP:
        //left_turn function 
        robot.left_turn();
        //Just angular speed until -90 degrees (use encoders)
        break;

    case RIGHT_TURN_MAP:
        //right_turn function
        robot.right_turn();
        //Just angular speed until 90 degrees (use encoders)
        break;

    case REVERSE:
        //reverse function 
        robot.reverse();
        //activate motors in oposite until see line (in this case, since its always turning left, it will see either OOXXX or OOXOO)
        break;

    case SMALL_FORWARD:
        //small_forward function (maybe just forward function but with timer)
        robot.small_forward();
        break;

    case FORWARD_MAP:
        //forward function (go forward until OOXOO)
        robot.forward();
        break;

    case END:
       // solve_algorithm();
        robot.stop();
        break;

    default:
        break;
    }
}











#ifdef SOLVE_MAP
void Solve_FSM_Handler()
{
  	switch (robot.currentStateSolve)
		{

			case IDLE_SOLVE:
      
        #ifdef DEBUG
        printf("-- Current state solve = IDLE\n");
        #endif

        if(robot.currentStateMain == SOLVE)
        {
          robot.currentStateSolve = FOLLOW_LINE_SOLVE;
        }

			break;




			case FOLLOW_LINE_SOLVE:

        #ifdef DEBUG
        printf("-- Current state solve = FOLLOW_LINE\n");
        #endif

        node = robot.IRLine.detectNode();
        if(node == robot.past_node){
          robot.node_count++;
        } else {
          robot.node_count = 0;
          robot.past_node = node;
        }
        if(robot.node_count < NODE_DETECTION){
          robot.currentStateSolve = FOLLOW_LINE_SOLVE;
        }
        else{
          Serial.print("Stable node detected: ");
          Serial.println(node);
          if(node == 'N'){
            robot.currentStateSolve = FOLLOW_LINE_SOLVE;
          }else{
            robot.currentStateSolve = GET_INSTRUCTION;
          }
          
        }

			  break;

			case GET_INSTRUCTION:

        #ifdef DEBUG
        printf("-- Current state solve = GET_INSTRUCTION\n");
        #endif
        node = robot.IRLine.detectNode();
        if(node == 'B'){
          robot.currentStateSolve = FINISH;
          break;
        }
        if(!robot.node_stack.empty())
        {
          char instruction = robot.node_stack.front();
          robot.node_stack.erase(robot.node_stack.begin());
          if(instruction == 'F'){
            robot.currentStateSolve = FORWARD_SOLVE;
          }else if(instruction == 'R'){
            robot.currentStateSolve = RIGHT_TURN_SOLVE;
          }else if(instruction == 'L'){
            robot.currentStateSolve = LEFT_TURN_SOLVE;
          }else if(instruction == 'U'){
            robot.currentStateSolve = FINISH; //U-turns arent allowed in solve phase
          }
        }
        else
        {
          // No more instructions, go to FINISH
          robot.currentStateSolve = FINISH;
        }

			break;

			case RIGHT_TURN_SOLVE:

        #ifdef DEBUG
        printf("-- Current state solve = RIGHT_TURN_SOLVE\n");
        #endif
        robot.currentStateSolve = FOLLOW_LINE_SOLVE;

			break;



    case LEFT_TURN_SOLVE:

        #ifdef DEBUG
        printf("-- Current state solve = LEFT_TURN_SOLVE\n");
        #endif
        robot.currentStateSolve = FOLLOW_LINE_SOLVE;

      break;

      case FORWARD_SOLVE:

        #ifdef DEBUG
        printf("-- Current state solve = FORWARD_SOLVE\n");
        #endif
        robot.currentStateSolve = FOLLOW_LINE_SOLVE;

      break;

      case FINISH:

        #ifdef DEBUG
        printf("-- Current state solve = FINISH\n");
        #endif

        if(0)
        {
          robot.currentStateSolve = IDLE_SOLVE;
        }

      break;

      }




 //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------     
// Update outputs
switch(robot.currentStateSolve)
    {
    case IDLE_SOLVE:
        break;

    case FOLLOW_LINE_SOLVE:
        robot.followLine();
        break;

    case GET_INSTRUCTION:
        robot.small_forward();
        break;

    case RIGHT_TURN_SOLVE:
        robot.right_turn();
        break;

    case LEFT_TURN_SOLVE:
        robot.left_turn();
        break;

    case FORWARD_SOLVE:
        robot.small_forward();
        break;

    case FINISH:
        robot.stop();
        break;

    default:
        break;
    }

}

#endif
#endif


/*
void Test_FSM_Handler()
{
    // Declare 'type_of_node' once, outside the switch statement
    char type_of_node = robot.IRLine.detectNode();
    // Debugging output
    // Serial.printf("RE_START_BUTTON %d\n", re_START_BUTTON);
    // Serial.println();

    static bool pushedR = false;  // Flag for Right turn state
    static bool pushedL = false;  // Flag for Left turn state
    static bool pushedU = false;  // Flag for U-turn state
    static bool pushedF = false;  // Flag for Forward state


    switch (currentStateTest)
    {
      case FOLLOW_TEST:
          #ifdef DEBUG
          Serial.print("-- Current state test = FOLLOW \n");
          #endif

          // Reset push flags so each state can push only once during its activation
          // pushedR = false;
          // pushedL = false;
          // pushedU = false;
          // pushedF = false;

          if (re_START_BUTTON == 1)
          {
            #ifdef SUPERDEBUG
          Serial.printf("-- Current state test =  FOLLOW_TEST (re_START_BUTTON) \n");
          #endif
              currentStateTest = STOP_TEST;
          }


          if (type_of_node == 'B')
          {

              #ifdef SUPERDEBUG
              Serial.printf("-- Current state test = FOLLOW_TEST (R) \n");
              #endif

              forwardStartTime = 0;
              cross = true;
              currentStateTest = SMALL_FORWARD_TEST;
          } 
          else if (type_of_node == 'L') {

              #ifdef SUPERDEBUG
              Serial.printf("-- Current state test = FOLLOW_TEST (L) \n");
              #endif
              //path_taken.push('L');
              currentStateTest = LEFT_TURN_TEST;
          } 
          else if (type_of_node == 'W') {

              #ifdef SUPERDEBUG
              Serial.printf("-- Current state test = FOLLOW_TEST (W) \n");
              #endif
              currentStateTest = U_TURN_TEST;
          } 
          else if(type_of_node == 'R')
          {
              #ifdef SUPERDEBUG
              Serial.printf("-- Current state test = FOLLOW_TEST (B) \n");
              #endif
               forwardStartTime = 0;
              currentStateTest =SMALL_FORWARD_TEST;
          }
          break;
          
      case FORWARD_TEST:
        #ifdef DEBUG
        Serial.print("-- Current state test = FORWARD \n");
        #endif

        if(re_START_BUTTON == 1)
        {

          #ifdef SUPERDEBUG
          Serial.printf("-- Current state test = FORWARD_TEST \n");
          #endif
          currentStateTest = STOP_TEST;
        }
        else if(type_of_node == 'N')  // Black, End of path
        { 
          if(!pushedF)
          {
            //path_taken.push('F');
            //Serial.println("Pushed F");
            pushedF = true;
          }

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = FORWARD_TEST \n");
            #endif
            currentStateTest = FOLLOW_TEST;
        }
        break;

         case SMALL_FORWARD_TEST:
        #ifdef DEBUG
        Serial.print("-- Current state test = SMALL_FORWARD\n");
        #endif

        // Check if enough time has passed or if the robot has moved a small distance.
        static bool hasMovedEnough = false;

        if (forwardStartTime == 0) 
        {
            // First time entering SMALL_FORWARD_TEST, start the timer
            forwardStartTime = millis();
        }

        // Check if the robot has moved forward long enough (adjust time as necessary)
        if (millis() - forwardStartTime > 200) { // 1000 ms = 1 second, adjust this as needed
            hasMovedEnough = true;
        }

        if (hasMovedEnough) 
        {
            // Check the node type after moving a little further
          
              if (type_of_node == 'B')  // Black, End of path
            { 
                currentStateTest = END_TEST;
            }
            else if (type_of_node == 'N')  // Cross or Right T junction
            { 
              currentStateTest = FOLLOW_TEST;

               if(cross == true)
               {
                cross = false;
                currentStateTest = BACKWARD_TEST;
               }
                
            }
            else if (type_of_node == 'W')  // White, U-turn
            { 
                currentStateTest = BACKWARD_TEST;
            }

            forwardStartTime = 0;  // Reset the timer
            hasMovedEnough = false; // Reset the movement condition
        }

        break;

      case RIGHT_TURN_TEST:
        #ifdef DEBUG
        Serial.printf("-- Current state test = RIGHT_TURN \n");
        #endif
        
        if (re_START_BUTTON == 1)
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = RIGHT_TURN_TEST \n");
            #endif
            currentStateTest = STOP_TEST;
        }

       else  if (type_of_node == 'N' && robot.END_TURN)  // Turn finished
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = RIGHT_TURN_TEST \n");
            #endif
            currentStateTest = FOLLOW_TEST;
            robot.END_TURN = false;
        }
        break;

      case LEFT_TURN_TEST:
        #ifdef DEBUG
        Serial.printf("-- Current state test = LEFT_TURN \n");
        #endif
       
        if (re_START_BUTTON == 1)
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = LEFT_TURN_TEST \n");
            #endif
            currentStateTest = STOP_TEST;
        }

        else if (type_of_node == 'N' && robot.END_TURN)  // Turn finished
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = LEFT_TURN_TEST \n");
            #endif
            currentStateTest = FOLLOW_TEST;
            robot.END_TURN = false;
        }
        break;

      case U_TURN_TEST:

        #ifdef DEBUG
        Serial.print("-- Current state test = U_TURN \n");
        #endif
        
        if (re_START_BUTTON == 1)
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = U_TURN_TEST (start_button)\n");
            #endif
            currentStateTest = STOP_TEST;
        }

        else if (type_of_node == 'N' && robot.END_TURN) // U-turn finished
        {
            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = U_TURN_TEST (end_turn) \n");
            #endif
            currentStateTest = FOLLOW_TEST;
            robot.END_TURN = false;
        }
        break;

      case BACKWARD_TEST:
        #ifdef DEBUG
        Serial.print("-- Current state test = BACKWARD \n");
        #endif

        type_of_node = robot.IRLine.detectNode();
        if (re_START_BUTTON == 1)
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = BACKWARD_TEST \n");
            #endif
            currentStateTest = STOP_TEST;
        }

        else if (type_of_node == 'R')  // Reversing until find right turn
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = BACKWARD_TEST \n");
            #endif
            currentStateTest = RIGHT_TURN_TEST;
        }

        else if (type_of_node == 'B')  // Reversing until find T junction
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = BACKWARD_TEST \n");
            #endif
            currentStateTest = LEFT_TURN_TEST;
        }
    
        break;

      case STOP_TEST:
        #ifdef DEBUG
        Serial.printf("-- Current state test = STOP  \n");
        #endif

        robot.END_TURN = false;

        if (re_START_BUTTON == 1)
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = STOP_TEST \n");
            #endif
            currentStateTest = FOLLOW_TEST;
        }

        break;

      case END_TEST:
        #ifdef DEBUG
        Serial.printf("-- Current state test = END \n");
        #endif

        robot.END_TURN = false;

        if (re_START_BUTTON == 1)
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = END_TEST \n");
            #endif
            currentStateTest = FOLLOW_TEST;
        }

        break;
    }



    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=



    // Update outputs
    switch (currentStateTest)
    {
      case FOLLOW_TEST:
        robot.followLine();
        break;

      case FORWARD_TEST:
        robot.forward();
        break;

      case SMALL_FORWARD_TEST:
        robot.forward();
        break;

      case RIGHT_TURN_TEST:
        robot.right_turn();
        break;

      case LEFT_TURN_TEST:
        robot.left_turn();
        break;

      case U_TURN_TEST:
        robot.u_turn();
        break;

      case BACKWARD_TEST:
        robot.reverse();
        break;

      case STOP_TEST:
        robot.stop();
        break;
      case END_TEST:
        // #ifdef DEBUG
        // Serial.printf("-- Current state test = END \n");
        // #endif

        robot.stop();
        solved_path = get_path(path_taken);  // Process the path once the test is over
        Serial.println("Taken Path: ");
        while (!path_taken.empty()) {
            char c = path_taken.top();
            Serial.print(c);
            path_taken.pop();
        }
        break;
    }
}*/


void FodaseFMSHandler(robot_t &robot)
{
  switch(robot.currentStatefodase)
  {
    case FOLLOW_FODASE:
      robot.followLine();
    break;
  }
}



