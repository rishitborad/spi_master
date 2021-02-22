#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <stdint.h>


typedef void(*tShellCallback)(uint32_t);

typedef struct {
    char const     *name;
    tShellCallback callback;
    uint32_t       callbackData;
    char const     *help;
} tCommand;

extern const tCommand gCommands[];

void InitCommandLine(void);
void CmdLineLookup(tCommand const *cmd_table);
int  CmdLineGetArgString(uint8_t **s);
int  CmdLineGetArgInt(int32_t *i);
int  CmdLineGetArgUInt(uint32_t *i);
int  CmdLineGetArgFloat(float *f);
void CmdLineExec(char const *cmdline, const tCommand *cmd_table);


#endif //COMMAND_LINE_H