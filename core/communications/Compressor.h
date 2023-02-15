#pragma once

#include <zlib.h>
#include <stdio.h>
#define MAX_COMPRESSED_LENGTH 5'000'000

template<int N = MAX_COMPRESSED_LENGTH>
class Compressor {
  public:
    template<typename T>
    inline std::vector<unsigned char> compress(const T& t) {
      unsigned long clen = buffer_.size();
      int res = compress(t, buffer_.data(), clen);
      if(res != Z_OK) {
        return std::vector<unsigned char>();
      }
      return std::vector<unsigned char>(buffer_.begin(), buffer_.begin() + clen);
    }
    
    template<typename T, typename U, typename V, typename = std::enable_if_t<sizeof(U) == 1 && std::is_integral<V>::value>>
    inline int compress(const T& t, U* buffer, V& length) {
      unsigned long clen = static_cast<unsigned long>(length);
      int res = compress2(static_cast<unsigned char*>(buffer), &clen, reinterpret_cast<const unsigned char*>(&t), sizeof(T), 3);
      length = clen;
      if(res != Z_OK)
        fprintf(stderr, "Error compressing data: %i\n", res);
      return res;
    }
    
    template<typename T>
    inline T decompress(const unsigned char* data, std::size_t n) {
      bool valid;
      decompress(data, n, valid);
    }

    template<typename T>
    inline T decompress(const unsigned char* data, std::size_t n, bool& valid) {
      unsigned long clen = buffer_.size();
      int res = uncompress(buffer_.data(), &clen, data, n);
      if(res != Z_OK) {
        valid = false;
        fprintf(stderr, "Error decompressing data: %i\n", res);
        return T();
      }
      T t;
      memcpy(reinterpret_cast<unsigned char*>(&t), buffer_.data(), clen);
      valid = true;
      return t;
    }

  private:
      std::array<unsigned char, N> buffer_;
};
