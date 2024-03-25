#pragma once

#include "Errors.h"

class Updatable
{
public:
    ERROR_TYPE virtual update() = 0;
};
