extern "C"
{

#ifdef _WINDOWS_
int __declspec(dllexport) __stdcall FindOneWire()
#else
int __attribute__((stdcall)) FindOneWire()
#endif
{
  return 123;
}

#ifdef _WINDOWS_
__declspec(dllexport) char * __stdcall GetOneWireString()
#else
char* __attribute__((stdcall)) GetOneWireString()
#endif
{
  return "1-wire string";
}

};
