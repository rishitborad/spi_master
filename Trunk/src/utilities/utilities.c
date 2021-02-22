/** ***************************************************************************
 * @file   utilities.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * string functions
 ******************************************************************************/
#include "utilities.h"

/** ****************************************************************************
 * @name: tolower
 * @brief reimplementation of library function:
 *        change uppercase letters to lowercase
 *  (This is just a helper function for strcmpi.)
 * @param [in] c - character of interest
 * @retval the character of interest (lowercase if it was upper)
 ******************************************************************************/
static char tolower(char c)
{
  if ('A' <= c && c <= 'Z') {
    return (c-'A')+'a';}

  return c;
}

/** ****************************************************************************
 * @name: strcmpi
 * @brief reimplementation of library function:
 *        compare two string without regard for case
 *
 * @param [in] two strings to be compared (a and b)
 * @retval strcmp returns 0 when the strings are equal, a negative
 *   integer when s1 is less than s2, or a positive integer if s1 is greater
 *   than s2, according to the lexicographical order.
 ******************************************************************************/
int strcmpi(char const *a,
            char const *b) {

  while ((*a) && (*b) && tolower(*a) == tolower(*b)) {
    a++;
    b++;
  }
  if (*a > *b) {
    return 1;
  } else if (*a < *b) {
    return -1;
  } else { //(*a == *b)
    return 0;
  }
}

/** ****************************************************************************
 * @name: strrep
 * @brief replace a character inside a string, for example to
 *   replace newline characters with string termination characters.
 *
 * @param [in] str - string to be modified
 * @param [in] find -  character to be replaced
 * @param [in] rep - replacement character
 * @retval None.
*******************************************************************************/
void strrep(char       *str,
            const char find,
            const char rep) {
	while (*str) {
		if (find == *str) {
			*str = rep;
		}
		str++;
	}
}

/** ****************************************************************************
 * @name: strtok_r
 * @brief reimplementation of library function:
 *        parses a string into a sequence of tokens
 *  The strtok_r() function is a reentrant version of strtok().
 *  It gets the next token from string s1, where tokens are strings separated
 *  by characters from s2. To get the first token from s1, strtok_r() is
 *  called with s1 as its first parameter. Remaining tokens from s1 are
 *  obtained by calling strtok_r() with a null pointer for the first
 *  parameter. The string of delimiters, s2, can differ from call to call.
 *
 *  @param [in] str - string to be searched
 *  @param [in] token - to use for separation
 *  @param [in] cursor - points to the last found token
 *  @retval returns the next token to be used.
*****************************************************************************/
char* strtok_r(char *str,
               char const token,
               char **cursor) {
	char *first;
	char *last;

	/// On the first call, we initialize from str
	/// On subsequent calls, str can be NULL
	if (str) {
		first = str;
	} else {
		first = *cursor;
		if (!first) {
		  return 0;
		}
	}
	/// Skip past any initial tokens
	while (*first && (*first == token)) {
		first++;
	}
	/// If string ends before first token is found, abort
	if (0 == *first) {
		return 0;
	}

	/// Now start moving last
	last = first + 1;
	while (*last && (*last != token)) {
		last++;
	}
	/// Found either end-of-token or end of string
	if (*last) {
		/// End of token
		*cursor = last + 1;
		*last = 0;
	} else {
		*cursor = 0;
	}
	return first;
}

char *strcpy(char *s1, const char *s2)
{       /* copy char s2[] to s1[] */
    char *s = s1;

    for (s = s1; (*s++ = *s2++) != '\0'; )
        ;
    return (s1);
}
