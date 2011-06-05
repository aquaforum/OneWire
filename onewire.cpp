extern "C"
{

int __declspec(dllexport) __stdcall FindOneWire()
{
  return 123;
}

__declspec(dllexport) char * __stdcall GetOneWireString()
{
  return "1-wire string";
}

};