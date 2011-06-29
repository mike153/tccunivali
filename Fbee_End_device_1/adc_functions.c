unsigned int media_amostra()
{
	unsigned int i;
	unsigned long valor_canal = 0;

	for(i = 0; i <256; i++) {
		ConvertADC();
		while(BusyADC()); //Espera a leitura
		valor_canal += ReadADC();
	}
	return valor_canal / 256; //media de 256
}

float le_temperatura()
{
	float tensao;
	unsigned int lido = media_amostra();
	tensao = (lido*(5000/1023)); // Vsinal = V_1bit * Vconversor
	return (int) tensao/10;
}