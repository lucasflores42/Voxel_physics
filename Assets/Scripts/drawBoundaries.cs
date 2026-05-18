using UnityEngine;

public class DesenharCaixa : MonoBehaviour
{
    // Criamos uma referência direta para o seu manager
    private SimulationManager manager;

    void OnDrawGizmos()
    {
        // Tenta encontrar o manager na cena se ele ainda não foi pego
        if (manager == null)
        {
            manager = FindFirstObjectByType<SimulationManager>();
        }

        // Se não encontrar o manager, não desenha nada para evitar erros
        if (manager == null) return;

        // Pega o tamanho correto direto do Inspector do Manager (funciona antes e depois do Play!)
        float tamanho = manager.boxSizeInspector;

        Gizmos.color = Color.red;

        // SE A SUA SIMULAÇÃO COMEÇA EM (0,0,0) E VAI ATÉ (boxSize):
        // O centro perfeito do cubo precisa ser a metade do tamanho em todos os eixos
        Vector3 centroPerfeito = new Vector3(tamanho / 2f, tamanho / 2f, tamanho / 2f);
        Vector3 tamanhoCubo = new Vector3(tamanho, tamanho, tamanho);

        // Desenha a caixa no lugar exato da simulação física
        Gizmos.DrawWireCube(centroPerfeito, tamanhoCubo);
    }
}
