typedef struct {
    
    
    
    


void sha256_init( SHA256_CTX * ctx );
void sha256_update( SHA256_CTX * ctx, char data[], uint32_t len );
void sha256_final( SHA256_CTX * ctx, uint8_t hash[] );