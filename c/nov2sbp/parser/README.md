# Novatel Binary Message Parser
Provide functionality for decoding Novatel binary messages into 
structs of data. Intended for internal use only.

## Adding a new message.
To add a new message using the current infrastructure, you must do the following:

1. Modify "message.h".
    - Add a new message identifier to the enum.

2. Write "message_<yourmessage>.(h|cc)".
    - Add a struct type which has all of the fields in the message. 
      See the rangecmp message for a variable length example.
    - Write a function for converting an array of bytes into the struct type.
      The convention adopted here is to add a method on the struct called "FromBytes()".

2. Modify "parser.cc".
    - In the "::parse_bytes()" function, add a new case to the switch statement.
    
## Design notes.
- Some helper functionality is implemented only in the headers.
- Although it seems like the message implementation would benefit from some object-oriented design,
  in practice it actually results in more confusing code. Using a simple switch statement, message ID,
  and sticking to the convention of implementing a "FromBytes()" function for each message type ends
  up being more readable and doesn't really cost any extra programmer time.

