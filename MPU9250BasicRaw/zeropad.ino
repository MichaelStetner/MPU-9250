// Converts a number, x, to string and then adds zeros to the left side until it is of length len.
// Finally, if x is negative, the first character will be "-". Otherwise the first character will
// be " ".
String zeropad(long x, int len)
{
  String str = String(abs(x));
  while (str.length() < len) {
    str = "0" + str;
  }
  if (x < 0) {
    return "-" + str;
  } else {
    return " " + str;
  }
}
