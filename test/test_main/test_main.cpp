#include <Arduino.h>
#include <unity.h>

// Simples função de exemplo a ser testada
int soma(int a, int b) {
    return a + b;
}

// Função de setup para o Unity
void setUp() {
    // Configurações iniciais para os testes (se necessário)
}

// Função de teardown para o Unity
void tearDown() {
    // Código para limpar testes (se necessário)
}

// Teste 1: Verificar se soma está correta
void test_soma() {
    TEST_ASSERT_EQUAL(5, soma(2, 3));
    TEST_ASSERT_EQUAL(-1, soma(2, -3));
}

// Teste 2: Verificar se ESP32 está rodando
void test_esp32_running() {
    TEST_ASSERT_TRUE(true);
}

void setup() {
    // Inicializa a serial (necessário para Unity no ESP32)
    Serial.begin(115200);
    
    // Aguarda para garantir que a serial está inicializada
    delay(2000);

    UNITY_BEGIN();  // Inicia o framework de testes

    RUN_TEST(test_soma);
    RUN_TEST(test_esp32_running);

    UNITY_END();  // Finaliza os testes
}

void loop() {
    // Não precisa de loop para testes unitários
}
