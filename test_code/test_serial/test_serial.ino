#define MAX_LEN 500
#define MAX_N_PARAMS 20
#define MAX_COMMAND_LEN 20

int cur_idx = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}


void loop() {
  // put your main code here, to run repeatedly:

  char write_buffer[MAX_LEN];
  int done;

  if (Serial.available() >= 1) {
    done = sRead(write_buffer, &cur_idx);
  }

  if (done) {

    done = 0;
    cur_idx = 0;
    parse_command(&write_buffer[0]);
    
  }
  
}


/*  Parses a character array command up to MAX_LEN
 *  characters long, formatted 'COMMAND:PARAM1:PARAM2:...:PARAMN', and
 *  executes a corresponding function.
 *  
 *  Commands can can contain, at most, MAX_N_PARAM parameters. Additionally,
 *  no command should entirely contain the name of another (e.g. MOVE and MOVE_FORWARD).
 */
void parse_command(char* command_ptr) {

  char command[MAX_COMMAND_LEN];
  char* params[MAX_N_PARAMS];
  int cur_param = 0;
  char cur_char;
  int break_flag = 0;

  //  Break up string into parameters, separated by colons and terminated
  //  with a semicolon. Store as an array of pointers.
  for (int i = 0; ; i++) {

    if (break_flag) break;
    cur_char = *(command_ptr + sizeof(char)*i);
    
    switch (cur_char) {

      case ':':
        if (cur_param == 0) command[i] = cur_char;
        params[cur_param] = &command_ptr[i+1];
        cur_param++;
        break;

      case ';':
        break_flag = 1;

      default:
        if (cur_param == 0) command[i] = cur_char;

    }
  }


  ////////////////////////////////////////////////////////
  //               PARSING FUNCTION CALLS               //
  ////////////////////////////////////////////////////////

  if (strncmp(command, "MOVE", 4) == 0) sMove(s2i(params[0]));
  else if (strncmp(command, "ROTATE", 6) == 0) sRotate(s2d(params[0]));
  else if (strncmp(command, "LED", 3) == 0) sLED(s2i(params[0]));


  ////////////////////////////////////////////////////////
  //                 PRINT DEBUGGING                    //
  ////////////////////////////////////////////////////////

//  // Prints the command name, number of arguments, and each argument in order.
//  int num_args = cur_param;
//  Serial.print("COMMAND: ");
//  for (int i=0; ; i++) {
//    if (command[i] == ':' || command[i] == ';') break;
//    Serial.print(command[i]);
//  }
//  Serial.println();
//  Serial.print("NUMBER OF ARGUMENTS: ");
//  Serial.println(num_args);
//  int idx;
//  char new_char;
//  for (int i = 0; i < num_args; i++) {
//
//    Serial.print("ARGUMENT ");
//    Serial.print(i);
//    Serial.print(": |");
//    idx = 0;
//    while (true) {
//      new_char = *(params[i] + sizeof(char)*idx);
//      if (new_char == ':' || new_char == ';') break;
//      Serial.print(new_char);
//      idx++;
//    }
//    Serial.println('|');
//
//  }
//  Serial.println();

}

/*  Example function to test serial communication that prints an 
 *  enthusiastic message whever called.
 */
int sMove(int distance) {
}

int sRotate(double amt) {
}

int sLED(int on) {
  if (on) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}


/*  Reads a character from Serial to the write buffer at
 *  an index specified by a pointer and returns 0, ignoring
 *  newline characters.
 *  
 *  If a semicolon is detected, returns 1.
 *  
 *  Increments the integer of the index pointer with
 *  every call.
 */
int sRead(char* write_buffer, int* idx) {

  //  Add character to the write buffer
  char read_value = (char)Serial.read();

  switch (read_value) {

    // Semicolons indicate end of message
    case ';':
      write_buffer[*idx] = read_value;
      (*idx)++;
      for (int j = 0; ; j++) {
        if (write_buffer[j] == ';') break;
      }

      return 1;

    // All other characters are written to write buffer
    default:
      write_buffer[*idx] = read_value;

      // DO NOT REMOVE THE FOLLOWING LINE OF CODE.
      // Otherwise, the compiler will optimize away the
      // modified value to write_buffer, and this
      // function will not work as intended.
      if (write_buffer[*idx] == ';') break; //  This should never happen

      (*idx)++;
      
      return 0;
  }
  
}

/*  Converts a string from a string pointer, ending with a colon or semicolon, into an integer value.
 *  
 *  Ignores decimal points when scanning for digits. E.g.) "30.25" becomes 3025.
 *  
 *  Negative values work, provided the negative sign is the first character of the string and no other
 *  negative signs appear in the string.
 */
int s2i(char* str_ptr) {

  char new_char;
  int return_integer = 0;
  int sign = 1;

  // Iterate over pointer string
  for (int i = 0; ; i++) {

    new_char = *(str_ptr + sizeof(char)*i);

    switch (new_char) {

      // Incorporate numeric values into return integer
      case '0': case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
      
        return_integer *= 10;
        return_integer += c2i(new_char);
        break;

      // End on colon or semicolon
      case ':': case ';':
      
        return return_integer * sign;

      case '-':

        sign *= -1;
        break;
    }
  }
}

/* Converts a character '0'-'9' to the corresponding integer value.
 * Returns -1 for all non-digit arguments.
 */
int c2i(char c) {
  switch (c) {
    case '0': return 0;
    case '1': return 1;
    case '2': return 2;
    case '3': return 3;
    case '4': return 4;
    case '5': return 5;
    case '6': return 6;
    case '7': return 7;
    case '8': return 8;
    case '9': return 9;
    default:  return -1;
  }
}

/* Converts a string from a string pointer, ending with a colon or semicolon, into a double value.
 *  
 * Ignores all but the last decimal point in arguments. Any minus signs in argument multiply the answer
 * by -1 (so, it works if the argument only contains a single one). Ignores all characters that aren't
 * numbers, end characters, decimal points, or hyphens.
 */
double s2d(char* str_ptr) {

  char new_char;
  double return_val = 0;
  double decimal_idx = -1.0;
  double pow10 = 0;
  double sign = 1;

  // Iterate over pointer string
  for (int i = 0; ; i++) {

    new_char = *(str_ptr + sizeof(char)*i);

    switch (new_char) {

      // Incorporate numeric values into return integer
      case '0': case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
      
        return_val *= 10.0;
        return_val += c2i(new_char);
        break;

      // End on colon or semicolon
      case ':': case ';':

        if (decimal_idx == -1) return return_val;
        pow10 = decimal_idx - i + 1;
        
        return return_val * pow(10.0, pow10) * sign;

      case '.': 
      
        decimal_idx = i;
        break;

      case '-':

        sign *= -1;
        break;
      
    }
  }
}
