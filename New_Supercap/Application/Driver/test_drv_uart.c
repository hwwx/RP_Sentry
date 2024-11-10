void test_USART1_Init(void)
{
	// Call the function to be tested
	USART1_Init();
	
	// Check if the IDLE flag is cleared
	TEST_ASSERT_EQUAL_UINT32(0, huart2.Instance->ISR & USART_ISR_IDLE);
	
	// Check if the IDLE interrupt is enabled
	TEST_ASSERT_EQUAL_UINT32(UART_IT_IDLE, READ_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE));
}