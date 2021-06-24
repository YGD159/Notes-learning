   #include <iostream>
   #include <fstream>
   #include "Mymessage.pb.h"

   void ListMsg(const Im::Content& msg)
   {
           std::cout << msg.id() << std::endl;
           std::cout << msg.str() << std::endl;
   }
  int main()
  {
          Im::Content msg1;
          std::fstream input("./log", std::ios::in|std::ios::binary);
          if(!msg1.ParseFromIstream(&input))
          {
                  std::cerr << "Failed to parse address book." << std::endl;
                  return -1;
          }
          ListMsg(msg1);
          return 0;
  }
