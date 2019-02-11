
/*
bool lineReady()
{
  bool          ready     = false;
  const char    endMarker = '\n';  // this is LF for Arduino, ASCII value 10

  while (Serial.available()) {

    char c = Serial.read();

    if (c != endMarker) {

      // Not the end, save another character.  But only
      //   only save the printable characters, and only
      //   if there's room.
      // This ignores CR characters (ASCII char value 13).
      //   Actually, this ignores all control characters:
      //   any thing less than a space char, ASCII value 32.

      if ((c >= ' ') and (count < MAX_CHARS-1)) {
        line[ count++ ] = c;
      }

    } else {
      //  It's the end marker, line is completely received

      line[count] = '\0'; // terminate the string
      count       = 0;    // reset for next time
      ready       = true;
      break;
    }
  }

  return ready;

} // lineReady
*/
