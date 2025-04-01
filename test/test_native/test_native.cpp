#include <unity.h>

// Simples função de exemplo para testar no ambiente "native"
int soma(int a, int b) {
    return a + b;
}

// Teste 1: Verifica se a soma funciona corretamente
void test_soma() {
    TEST_ASSERT_EQUAL(5, soma(2, 3));
    TEST_ASSERT_EQUAL(-1, soma(2, -3));
}

// Setup dos testes
void setUp() {}

// Teardown dos testes
void tearDown() {}

int main() {
    UNITY_BEGIN();  // Inicia os testes

    RUN_TEST(test_soma);

    UNITY_END();  // Finaliza os testes
    return 0;
}
