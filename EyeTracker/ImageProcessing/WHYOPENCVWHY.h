
static int EyeConvertToByte(char Value);
static char EyeConvertToChar(int Value);

int EyeConvertToByte(char Value)
{
    if (Value < 0)
    {
        return Value + 255;
    }
    return Value;
}

char EyeConvertToChar(int Value)
{
    if (Value > 127)
    {
        return Value - 255;
    }
    return Value;
}
