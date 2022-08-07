
#include "JC_helper_authenticate.hpp"

using namespace std;

#pragma region  //*获取mac地址

int getEthNames( set< string >& ethName )
{
    FILE* fp = NULL;
    char* p  = NULL;
    char linebuf[ 512 ];
    char devname[ 128 ];
    string tmp;

    fp = fopen( "/proc/net/dev", "r" );
    if ( fp == NULL )
    {
        return -1;
    }

    memset( linebuf, 0x00, sizeof( linebuf ) );
    memset( devname, 0x00, sizeof( devname ) );

    while ( fgets( linebuf, 511, fp ) != NULL )
    {
        p = strstr( linebuf, ":" );
        if ( p == NULL )
        {
            memset( linebuf, 0x00, sizeof( linebuf ) );
            continue;
        }

        p[ 0 ] = 0x00;

        memset( devname, 0x00, sizeof( devname ) );

        strncpy( devname, linebuf, 127 );

        tmp = string( devname );
        tmp.erase( 0, tmp.find_first_not_of( " " ) );
        tmp.erase( tmp.find_last_not_of( " " ) + 1 );

        if ( strncmp( tmp.c_str( ), "lo", 2 ) != 0 )
        {
            // if(strncmp(tmp.c_str(), "eth", 3) == 0 || strncmp(tmp.c_str(), "ens", 3) == 0 || \
            //     strncmp(tmp.c_str(), "enp", 3) == 0 || strncmp(tmp.c_str(), "en", 2) == 0 || \
            //     strncmp(tmp.c_str(), "wlp", 3) == 0 )
            // {
            //     ethName.insert(tmp);
            // }

            ethName.insert( tmp );
        }

        memset( linebuf, 0x00, sizeof( linebuf ) );
    }

    fclose( fp );

    return 0;
}

int getIPMACs( set< string >& ethName, map< string, pair< string, string > >& macs )
{
    set< string >::iterator it;
    struct ifreq ifr;
    struct sockaddr_in* sin;
    char ip_addr[ 30 ];
    char mac_addr[ 30 ];
    int sockfd = -1;
    int nRes   = -1;

    sockfd = socket( AF_INET, SOCK_STREAM, 0 );
    if ( sockfd < 0 )
    {
        return -1;
    }

    for ( it = ethName.begin( ); it != ethName.end( ); it++ )
    {
        nRes = -1;

        memset( ip_addr, 0x00, sizeof( ip_addr ) );
        memset( mac_addr, 0x00, sizeof( mac_addr ) );

        memset( &ifr, 0x00, sizeof( ifr ) );

        strcpy( ifr.ifr_name, ( *it ).c_str( ) );

        nRes = ioctl( sockfd, SIOCGIFADDR, &ifr );
        if ( nRes < 0 )
        {
            strcpy( ip_addr, "" );
        }
        else
        {
            sin = ( struct sockaddr_in* )&ifr.ifr_addr;
            strcpy( ip_addr, inet_ntoa( sin->sin_addr ) );
        }

        nRes = ioctl( sockfd, SIOCGIFHWADDR, &ifr );
        if ( nRes < 0 )
        {
            strcpy( mac_addr, "00:00:00:00:00:00" );
        }
        else
        {
            sprintf( mac_addr, "%02x:%02x:%02x:%02x:%02x:%02x",
                     ( unsigned char )ifr.ifr_hwaddr.sa_data[ 0 ],
                     ( unsigned char )ifr.ifr_hwaddr.sa_data[ 1 ],
                     ( unsigned char )ifr.ifr_hwaddr.sa_data[ 2 ],
                     ( unsigned char )ifr.ifr_hwaddr.sa_data[ 3 ],
                     ( unsigned char )ifr.ifr_hwaddr.sa_data[ 4 ],
                     ( unsigned char )ifr.ifr_hwaddr.sa_data[ 5 ] );
        }

        macs.insert( make_pair( string( mac_addr ), make_pair( string( ifr.ifr_name ), string( ip_addr ) ) ) );
    }

    close( sockfd );

    return 0;
}

#pragma endregion

#pragma region  //*base64

static const char* base64_chars[ 2 ] = {
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "+/",

    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "-_" };

class base64
{
public:
    //
    // Depending on the url parameter in base64_chars, one of
    // two sets of base64 characters needs to be chosen.
    // They differ in their last two characters.
    //

    static unsigned int pos_of_char( const unsigned char chr )
    {
        //
        // Return the position of chr within base64_encode()
        //

        if ( chr >= 'A' && chr <= 'Z' )
            return chr - 'A';
        else if ( chr >= 'a' && chr <= 'z' )
            return chr - 'a' + ( 'Z' - 'A' ) + 1;
        else if ( chr >= '0' && chr <= '9' )
            return chr - '0' + ( 'Z' - 'A' ) + ( 'z' - 'a' ) + 2;
        else if ( chr == '+' || chr == '-' )
            return 62;  // Be liberal with input and accept both url ('-') and non-url ('+') base 64 characters (
        else if ( chr == '/' || chr == '_' )
            return 63;  // Ditto for '/' and '_'
        else
            //
            // 2020-10-23: Throw std::exception rather than const char*
            //(Pablo Martin-Gomez, https://github.com/Bouska)
            //
            throw std::runtime_error( "Input is not valid base64-encoded data." );
    }

    static std::string insert_linebreaks( std::string str, size_t distance )
    {
        //
        // Provided by https://github.com/JomaCorpFX, adapted by me.
        //
        if ( !str.length( ) )
        {
            return "";
        }

        size_t pos = distance;

        while ( pos < str.size( ) )
        {
            str.insert( pos, "\n" );
            pos += distance + 1;
        }

        return str;
    }

    template < typename String, unsigned int line_length >
    static std::string encode_with_line_breaks( String s )
    {
        return insert_linebreaks( base64_encode( s, false ), line_length );
    }

    template < typename String >
    static std::string encode_pem( String s )
    {
        return encode_with_line_breaks< String, 64 >( s );
    }

    template < typename String >
    static std::string encode_mime( String s )
    {
        return encode_with_line_breaks< String, 76 >( s );
    }

    template < typename String >
    static std::string encode( String s, bool url )
    {
        return base64_encode( reinterpret_cast< const unsigned char* >( s.data( ) ), s.length( ), url );
    }

    static std::string base64_encode( unsigned char const* bytes_to_encode, size_t in_len, bool url = false )
    {
        size_t len_encoded = ( in_len + 2 ) / 3 * 4;

        unsigned char trailing_char = url ? '.' : '=';

        //
        // Choose set of base64 characters. They differ
        // for the last two positions, depending on the url
        // parameter.
        // A bool (as is the parameter url) is guaranteed
        // to evaluate to either 0 or 1 in C++ therefore,
        // the correct character set is chosen by subscripting
        // base64_chars with url.
        //
        const char* base64_chars_ = base64_chars[ url ];

        std::string ret;
        ret.reserve( len_encoded );

        unsigned int pos = 0;

        while ( pos < in_len )
        {
            ret.push_back( base64_chars_[ ( bytes_to_encode[ pos + 0 ] & 0xfc ) >> 2 ] );

            if ( pos + 1 < in_len )
            {
                ret.push_back( base64_chars_[ ( ( bytes_to_encode[ pos + 0 ] & 0x03 ) << 4 ) + ( ( bytes_to_encode[ pos + 1 ] & 0xf0 ) >> 4 ) ] );

                if ( pos + 2 < in_len )
                {
                    ret.push_back( base64_chars_[ ( ( bytes_to_encode[ pos + 1 ] & 0x0f ) << 2 ) + ( ( bytes_to_encode[ pos + 2 ] & 0xc0 ) >> 6 ) ] );
                    ret.push_back( base64_chars_[ bytes_to_encode[ pos + 2 ] & 0x3f ] );
                }
                else
                {
                    ret.push_back( base64_chars_[ ( bytes_to_encode[ pos + 1 ] & 0x0f ) << 2 ] );
                    ret.push_back( trailing_char );
                }
            }
            else
            {
                ret.push_back( base64_chars_[ ( bytes_to_encode[ pos + 0 ] & 0x03 ) << 4 ] );
                ret.push_back( trailing_char );
                ret.push_back( trailing_char );
            }

            pos += 3;
        }

        return ret;
    }

    template < typename String >
    static std::string decode( String encoded_string, bool remove_linebreaks )
    {
        //
        // decode(…) is templated so that it can be used with String = const std::string&
        // or std::string_view (requires at least C++17)
        //

        if ( encoded_string.empty( ) ) return std::string( );

        if ( remove_linebreaks )
        {
            std::string copy( encoded_string );

            copy.erase( std::remove( copy.begin( ), copy.end( ), '\n' ), copy.end( ) );

            return base64_decode( copy, false );
        }

        size_t length_of_string = encoded_string.length( );
        size_t pos              = 0;

        //
        // The approximate length (bytes) of the decoded string might be one or
        // two bytes smaller, depending on the amount of trailing equal signs
        // in the encoded string. This approximation is needed to reserve
        // enough space in the string to be returned.
        //
        size_t approx_length_of_decoded_string = length_of_string / 4 * 3;
        std::string ret;
        ret.reserve( approx_length_of_decoded_string );

        while ( pos < length_of_string )
        {
            //
            // Iterate over encoded input string in chunks. The size of all
            // chunks except the last one is 4 bytes.
            //
            // The last chunk might be padded with equal signs or dots
            // in order to make it 4 bytes in size as well, but this
            // is not required as per RFC 2045.
            //
            // All chunks except the last one produce three output bytes.
            //
            // The last chunk produces at least one and up to three bytes.
            //

            size_t pos_of_char_1 = pos_of_char( encoded_string[ pos + 1 ] );

            //
            // Emit the first output byte that is produced in each chunk:
            //
            ret.push_back( static_cast< std::string::value_type >( ( ( pos_of_char( encoded_string[ pos + 0 ] ) ) << 2 ) + ( ( pos_of_char_1 & 0x30 ) >> 4 ) ) );

            if ( ( pos + 2 < length_of_string ) &&  // Check for data that is not padded with equal signs (which is allowed by RFC 2045)
                 encoded_string[ pos + 2 ] != '=' &&
                 encoded_string[ pos + 2 ] != '.'  // accept URL-safe base 64 strings, too, so check for '.' also.
            )
            {
                //
                // Emit a chunk's second byte (which might not be produced in the last chunk).
                //
                unsigned int pos_of_char_2 = pos_of_char( encoded_string[ pos + 2 ] );
                ret.push_back( static_cast< std::string::value_type >( ( ( pos_of_char_1 & 0x0f ) << 4 ) + ( ( pos_of_char_2 & 0x3c ) >> 2 ) ) );

                if ( ( pos + 3 < length_of_string ) &&
                     encoded_string[ pos + 3 ] != '=' &&
                     encoded_string[ pos + 3 ] != '.' )
                {
                    //
                    // Emit a chunk's third byte (which might not be produced in the last chunk).
                    //
                    ret.push_back( static_cast< std::string::value_type >( ( ( pos_of_char_2 & 0x03 ) << 6 ) + pos_of_char( encoded_string[ pos + 3 ] ) ) );
                }
            }

            pos += 4;
        }

        return ret;
    }

    static std::string base64_decode( std::string const& s, bool remove_linebreaks = false )
    {
        return decode( s, remove_linebreaks );
    }

    static std::string base64_encode( std::string const& s, bool url = false )
    {
        return encode( s, url );
    }

    static std::string base64_encode_pem( std::string const& s )
    {
        return encode_pem( s );
    }

    static std::string base64_encode_mime( std::string const& s )
    {
        return encode_mime( s );
    }
};

#pragma endregion

#pragma region  //*openSSL

// ---- rsa非对称加解密 ---- //
#define KEY_LENGTH 2048            // 密钥长度
#define PUB_KEY_FILE "pubkey.pem"  // 公钥路径
#define PRI_KEY_FILE "prikey.pem"  // 私钥路径

/**
 * @brief 生成公密和私密
 *
 * @param savePrivateKeyFilePath
 * @param savePublicKeyFilePath
 * @return true
 * @return false
 */
int MakeRsaKeySSL( const char* savePrivateKeyFilePath, const char* savePublicKeyFilePath )
{
    int ret        = 0;
    RSA* keypair   = NULL;
    BIGNUM* bne    = NULL;
    BIO *bp_public = NULL, *bp_private = NULL;
    bool pass_ok = true;

    int bits        = 2048;
    unsigned long e = RSA_F4;

    // 1. generate rsa key
    bne = BN_new( );
    ret = BN_set_word( bne, e );
    if ( ret != 1 )
    {
        fprintf( stderr, "MakeLocalKeySSL    BN_set_word err \n" );
        pass_ok = false;
        goto free_all;
    }

    keypair = RSA_new( );
    ret     = RSA_generate_key_ex( keypair, bits, bne, NULL );
    if ( ret != 1 )
    {
        fprintf( stderr, "MakeLocalKeySSL    RSA_generate_key_ex err \n" );
        pass_ok = false;
        goto free_all;
    }

    // 2. save public key
    if ( savePublicKeyFilePath != NULL )
    {
        bp_public = BIO_new_file( savePublicKeyFilePath, "w+" );
        // 注意------生成第1种格式的公钥
        //   ret       =PEM_write_bio_RSAPublicKey(bp_public, r);
        // 注意------生成第2种格式的公钥（此处代码中使用这种）
        ret = PEM_write_bio_RSA_PUBKEY( bp_public, keypair );

        if ( ret != 1 )
        {
            fprintf( stderr, "MakeLocalKeySSL    PEM_write_bio_RSA_PUBKEY err \n" );
            pass_ok = false;
            goto free_all;
        }
    }

    // 3. save private key
    if ( savePrivateKeyFilePath != NULL )
    {
        bp_private = BIO_new_file( savePrivateKeyFilePath, "w+" );
        ret        = PEM_write_bio_RSAPrivateKey( bp_private, keypair, NULL, NULL, 0, NULL, NULL );

        if ( ret != 1 )
        {
            fprintf( stderr, "MakeLocalKeySSL    PEM_write_bio_RSAPrivateKey err \n" );
            pass_ok = false;
            goto free_all;
        }
    }

    // 4. free
free_all:

    BIO_free_all( bp_public );
    BIO_free_all( bp_private );
    RSA_free( keypair );
    BN_free( bne );
    if ( pass_ok )
        return 1;
    else
        return 0;
}

int MakeRsaKeySSL( std::string& out_pub_key, std::string& out_pri_key )
{
    size_t pri_len  = 0;        // 私钥长度
    size_t pub_len  = 0;        // 公钥长度
    char* pri_key   = nullptr;  // 私钥
    char* pub_key   = nullptr;  // 公钥
    int ret         = 0;
    RSA* keypair    = NULL;
    int bits        = 2048;
    unsigned long e = RSA_F4;
    BIGNUM* bne     = NULL;

    // 生成密钥对

    bne = BN_new( );
    ret = BN_set_word( bne, e );
    if ( ret != 1 )
    {
        fprintf( stderr, "MakeLocalKeySSL    BN_set_word err \n" );
        RSA_free( keypair );
        free( pri_key );
        free( pub_key );
        return -1;
    }

    keypair = RSA_new( );
    ret     = RSA_generate_key_ex( keypair, bits, bne, NULL );
    if ( ret != 1 )
    {
        fprintf( stderr, "MakeLocalKeySSL RSA_generate_key_ex err \n" );
        RSA_free( keypair );
        free( pri_key );
        free( pub_key );
        return -1;
    }

    BIO* pri = BIO_new( BIO_s_mem( ) );
    BIO* pub = BIO_new( BIO_s_mem( ) );

    // 生成私钥
    PEM_write_bio_RSAPrivateKey( pri, keypair, NULL, NULL, 0, NULL, NULL );
    // 注意------生成第1种格式的公钥
    // PEM_write_bio_RSAPublicKey(pub, keypair);
    // 注意------生成第2种格式的公钥（此处代码中使用这种）
    PEM_write_bio_RSA_PUBKEY( pub, keypair );

    // 获取长度
    pri_len = BIO_pending( pri );
    pub_len = BIO_pending( pub );

    // 密钥对读取到字符串
    pri_key = ( char* )malloc( pri_len + 1 );
    pub_key = ( char* )malloc( pub_len + 1 );

    BIO_read( pri, pri_key, pri_len );
    BIO_read( pub, pub_key, pub_len );

    pri_key[ pri_len ] = '\0';
    pub_key[ pub_len ] = '\0';

    out_pub_key = pub_key;
    out_pri_key = pri_key;

// 释放内存
free_all:

    RSA_free( keypair );
    BIO_free_all( pub );
    BIO_free_all( pri );

    free( pri_key );
    free( pub_key );

    return 0;
}

// 命令行方法生成公私钥对（begin public key/ begin private key）
// 找到openssl命令行工具，运行以下
// openssl genrsa -out prikey.pem 1024
// openssl rsa - in privkey.pem - pubout - out pubkey.pem

// 公钥加密
std::string rsa_pub_encrypt( const std::string& clearText, const std::string& pubKey )
{
    std::string strRet;
    RSA* rsa    = NULL;
    BIO* keybio = BIO_new_mem_buf( ( unsigned char* )pubKey.c_str( ), -1 );
    // 此处有三种方法
    // 1, 读取内存里生成的密钥对，再从内存生成rsa
    // 2, 读取磁盘里生成的密钥对文本文件，在从内存生成rsa
    // 3，直接从读取文件指针生成rsa
    rsa = PEM_read_bio_RSAPublicKey( keybio, &rsa, NULL, NULL );

    int len             = RSA_size( rsa );
    char* encryptedText = ( char* )malloc( len + 1 );
    memset( encryptedText, 0, len + 1 );

    // 加密函数
    int ret = RSA_public_encrypt( clearText.length( ), ( const unsigned char* )clearText.c_str( ), ( unsigned char* )encryptedText, rsa, RSA_PKCS1_PADDING );
    if ( ret >= 0 )
        strRet = std::string( encryptedText, ret );

    // 释放内存
    free( encryptedText );
    BIO_free_all( keybio );
    RSA_free( rsa );

    return strRet;
}
// 公钥加密
std::string rsa_pub_encrypt_from_file( const std::string& clearText, const std::string& pubKey_path )
{
    std::string strRet;
    RSA* rsa    = NULL;
    BIO* keybio = BIO_new( BIO_s_file( ) );
    ;
    BIO_read_filename( keybio, pubKey_path.c_str( ) );

    // 此处有三种方法
    // 1, 读取内存里生成的密钥对，再从内存生成rsa
    // 2, 读取磁盘里生成的密钥对文本文件，在从内存生成rsa
    // 3，直接从读取文件指针生成rsa
    rsa                 = PEM_read_bio_RSAPublicKey( keybio, &rsa, NULL, NULL );
    int len             = RSA_size( rsa );
    char* encryptedText = ( char* )malloc( len + 1 );
    memset( encryptedText, 0, len + 1 );

    // 加密函数
    int ret = RSA_public_encrypt( clearText.length( ), ( const unsigned char* )clearText.c_str( ), ( unsigned char* )encryptedText, rsa, RSA_PKCS1_PADDING );
    if ( ret >= 0 )
        strRet = std::string( encryptedText, ret );

    // 释放内存
    free( encryptedText );
    BIO_free_all( keybio );
    RSA_free( rsa );

    return strRet;
}

// 私钥解密
std::string rsa_pri_decrypt( const std::string& cipherText, const std::string& priKey )
{
    std::string strRet;
    RSA* rsa = RSA_new( );
    BIO* keybio;
    keybio = BIO_new_mem_buf( ( unsigned char* )priKey.c_str( ), -1 );

    // 此处有三种方法
    // 1, 读取内存里生成的密钥对，再从内存生成rsa
    // 2, 读取磁盘里生成的密钥对文本文件，在从内存生成rsa
    // 3，直接从读取文件指针生成rsa
    rsa = PEM_read_bio_RSAPrivateKey( keybio, &rsa, NULL, NULL );

    int len             = RSA_size( rsa );
    char* decryptedText = ( char* )malloc( len + 1 );
    memset( decryptedText, 0, len + 1 );

    // 解密函数
    int ret = RSA_private_decrypt( cipherText.length( ), ( const unsigned char* )cipherText.c_str( ), ( unsigned char* )decryptedText, rsa, RSA_PKCS1_PADDING );
    if ( ret >= 0 )
        strRet = std::string( decryptedText, ret );

    // 释放内存
    free( decryptedText );
    BIO_free_all( keybio );
    RSA_free( rsa );

    return strRet;
}
// 私钥解密
std::string rsa_pri_decrypt_from_file( const std::string& cipherText, const std::string& priKey_path )
{
    std::string strRet;
    RSA* rsa    = RSA_new( );
    BIO* keybio = BIO_new( BIO_s_file( ) );

    BIO_read_filename( keybio, priKey_path.c_str( ) );

    // 此处有三种方法
    // 1, 读取内存里生成的密钥对，再从内存生成rsa
    // 2, 读取磁盘里生成的密钥对文本文件，在从内存生成rsa
    // 3，直接从读取文件指针生成rsa
    rsa = PEM_read_bio_RSAPrivateKey( keybio, &rsa, NULL, NULL );

    int len             = RSA_size( rsa );
    char* decryptedText = ( char* )malloc( len + 1 );
    memset( decryptedText, 0, len + 1 );

    // 解密函数
    int ret = RSA_private_decrypt( cipherText.length( ), ( const unsigned char* )cipherText.c_str( ), ( unsigned char* )decryptedText, rsa, RSA_PKCS1_PADDING );
    if ( ret >= 0 )
        strRet = std::string( decryptedText, ret );

    // 释放内存
    free( decryptedText );
    BIO_free_all( keybio );
    RSA_free( rsa );

    return strRet;
}

/*
@brief : 私钥加密(对长度较短的数据)
@para  : clear_text  -[i] 需要进行加密的明文
         pri_key     -[i] 私钥
@return: 加密后的数据
**/
std::string RsaPriEncrypt( const std::string& clear_text, const std::string& pri_key )
{
    std::string encrypt_text;
    BIO* keybio = BIO_new_mem_buf( ( unsigned char* )pri_key.c_str( ), -1 );
    RSA* rsa    = RSA_new( );
    rsa         = PEM_read_bio_RSAPrivateKey( keybio, &rsa, NULL, NULL );
    if ( !rsa )
    {
        BIO_free_all( keybio );
        return std::string( "" );
    }

    // 获取RSA单次可以处理的数据的最大长度
    int len = RSA_size( rsa );

    // 申请内存：存贮加密后的密文数据
    char* text = new char[ len + 1 ];
    memset( text, 0, len + 1 );

    // 对数据进行私钥加密（返回值是加密后数据的长度）
    int ret = RSA_private_encrypt( clear_text.length( ), ( const unsigned char* )clear_text.c_str( ), ( unsigned char* )text, rsa, RSA_PKCS1_PADDING );
    if ( ret >= 0 )
    {
        encrypt_text = std::string( text, ret );
    }
    else
    {
        PLOG_ERROR << "RSA_private_encrypt error";
        BIO_free_all( keybio );
        return std::string( "" );
    }

    // 释放内存
    free( text );
    BIO_free_all( keybio );
    RSA_free( rsa );

    return encrypt_text;
}

/*
@brief : 私钥加密(对长度较短的数据)
@para  : clear_text  -[i] 需要进行加密的明文
         pri_key     -[i] 私钥
@return: 加密后的数据
**/
std::string RsaPriEncrypt_from_file( const std::string& clear_text, const std::string& priKey_path )
{
    std::string encrypt_text;
    BIO* keybio = BIO_new( BIO_s_file( ) );
    RSA* rsa    = RSA_new( );
    BIO_read_filename( keybio, priKey_path.c_str( ) );
    rsa = PEM_read_bio_RSAPrivateKey( keybio, &rsa, NULL, NULL );

    if ( !rsa )
    {
        PLOG_ERROR << "PEM_read_bio_RSAPrivateKey error";
        BIO_free_all( keybio );
        return std::string( "" );
    }

    // 获取RSA单次可以处理的数据的最大长度
    int len = RSA_size( rsa );

    // 申请内存：存贮加密后的密文数据
    char* text = new char[ len + 1 ];
    memset( text, 0, len + 1 );

    // 对数据进行私钥加密（返回值是加密后数据的长度）
    int ret = RSA_private_encrypt( clear_text.length( ), ( const unsigned char* )clear_text.c_str( ), ( unsigned char* )text, rsa, RSA_PKCS1_PADDING );
    if ( ret >= 0 )
    {
        encrypt_text = std::string( text, ret );
    }
    else
    {
        PLOG_ERROR << "RSA_private_encrypt error";
        BIO_free_all( keybio );
        return std::string( "" );
    }

    // 释放内存
    free( text );
    BIO_free_all( keybio );
    RSA_free( rsa );

    return encrypt_text;
}

/*
@brief : 公钥解密(对长度较短的数据)
@para  : cipher_text -[i] 加密的密文
         pub_key     -[i] 公钥
@return: 解密后的数据
**/
std::string RsaPubDecrypt( const std::string& cipher_text, const std::string& pub_key )
{
    std::string decrypt_text;
    BIO* keybio = BIO_new_mem_buf( ( unsigned char* )pub_key.c_str( ), -1 );
    RSA* rsa    = RSA_new( );

    // 注意--------使用第1种格式的公钥进行解密
    // rsa = PEM_read_bio_RSAPublicKey(keybio, &rsa, NULL, NULL);
    // 注意--------使用第2种格式的公钥进行解密（我们使用这种格式作为示例）
    rsa = PEM_read_bio_RSA_PUBKEY( keybio, &rsa, NULL, NULL );
    if ( !rsa )
    {
        unsigned long err    = ERR_get_error( );  //获取错误号
        char err_msg[ 1024 ] = { 0 };
        ERR_error_string( err, err_msg );  // 格式：error:errId:库:函数:原因
        printf( "err msg: err:%ld, msg:%s\n", err, err_msg );
        BIO_free_all( keybio );
        return decrypt_text;
    }

    int len    = RSA_size( rsa );
    char* text = new char[ len + 1 ];
    memset( text, 0, len + 1 );
    // 对密文进行解密
    int ret = RSA_public_decrypt( cipher_text.length( ), ( const unsigned char* )cipher_text.c_str( ), ( unsigned char* )text, rsa, RSA_PKCS1_PADDING );
    if ( ret >= 0 )
    {
        decrypt_text.append( std::string( text, ret ) );
    }

    // 释放内存
    delete text;
    BIO_free_all( keybio );
    RSA_free( rsa );

    return decrypt_text;
}
/*
@brief : 公钥解密(对长度较短的数据)
@para  : cipher_text -[i] 加密的密文
         pub_key     -[i] 公钥
@return: 解密后的数据
**/
std::string RsaPubDecrypt_from_file( const std::string& cipher_text, const std::string& pub_key_path )
{
    std::string decrypt_text;
    BIO* keybio = BIO_new( BIO_s_file( ) );
    RSA* rsa    = RSA_new( );

    BIO_read_filename( keybio, pub_key_path.c_str( ) );
    // 注意--------使用第1种格式的公钥进行解密
    // rsa = PEM_read_bio_RSAPublicKey(keybio, &rsa, NULL, NULL);
    // 注意--------使用第2种格式的公钥进行解密（我们使用这种格式作为示例）
    rsa = PEM_read_bio_RSA_PUBKEY( keybio, &rsa, NULL, NULL );
    if ( !rsa )
    {
        unsigned long err    = ERR_get_error( );  //获取错误号
        char err_msg[ 1024 ] = { 0 };
        ERR_error_string( err, err_msg );  // 格式：error:errId:库:函数:原因
        printf( "err msg: err:%ld, msg:%s\n", err, err_msg );
        BIO_free_all( keybio );
        return decrypt_text;
    }

    int len    = RSA_size( rsa );
    char* text = new char[ len + 1 ];
    memset( text, 0, len + 1 );
    // 对密文进行解密
    int ret = RSA_public_decrypt( cipher_text.length( ), ( const unsigned char* )cipher_text.c_str( ), ( unsigned char* )text, rsa, RSA_PKCS1_PADDING );
    if ( ret >= 0 )
    {
        decrypt_text.append( std::string( text, ret ) );
    }

    // 释放内存
    delete text;
    BIO_free_all( keybio );
    RSA_free( rsa );

    return decrypt_text;
}

#pragma endregion



namespace JC_helper
{
    int authentication( )
    {
        std::ifstream licence_file;
        licence_file.open( "license" );
        if ( !licence_file.is_open( ) )
        {
            std::cout << RED << "license 文件不存在" << WHITE << std::endl;
            return -1;
        }

        char licence_data[ 10240 ];
        licence_file.getline( licence_data, sizeof( licence_data ),'\0' );
        licence_file.close( );


        std::string decoded = base64::base64_decode( std::string{licence_data} );
        // PLOG_DEBUG.printf( "base64解码:  :%s", decoded.c_str( ) );

        std::string decryptText = RsaPubDecrypt_from_file( decoded, "public.key" );
        // PLOG_DEBUG << "ssl解密:" << decryptText << std::endl;

        set< string > eth;
        map< string, pair< string, string > > macs;
        map< string, pair< string, string > >::iterator it;

        getEthNames( eth );
        getIPMACs( eth, macs );

        for ( it = macs.begin( ); it != macs.end( ); it++ )
        {
            // cout << ( *it ).first << " ";
            // cout << ( *it ).second.first << " ";
            // cout << ( *it ).second.second << endl;
            if ( decryptText == ( *it ).first )
            {
                std::cout << GREEN << base64::base64_decode( "PT09PT09PT09PT09PT09PT09PT09IOiupCDor4Eg6YCaIOi/hyA9PT09PT09PT09PT09PT09PT09PT0=") << WHITE << std::endl;
                return 0;
            }
        }

        std::cout << RED <<base64::base64_decode( "PT09PT09PT09PT09PT09PT09PT09IOiupCDor4Eg5aSxIOi0pSA9PT09PT09PT09PT09PT09PT09PT0=")  << WHITE << std::endl;
        std::cout << RED <<base64::base64_decode( "PT09Peivt+iBlOezu+euoeeQhuS6ujogbHVveWFuZ0BzaWEuY24g5oiWIDE5MzM1NTc5N0BxcS5jb209PT0=")  << WHITE << std::endl;

        return -1;
    }
}  // namespace JC_helper