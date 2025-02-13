#include <stdio.h>

#ifndef LOGGING_H
#define LOGGING_H

void setupLoggingFile();
void logVehicleModel();

extern FILE *outputFile;

#endif