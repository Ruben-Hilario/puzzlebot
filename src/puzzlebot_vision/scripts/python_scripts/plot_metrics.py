import pandas as pd
import matplotlib.pyplot as plt

def generar_graficas_rendimiento(csv_filename):
    try:
        df = pd.read_csv(csv_filename)
        df['segundos'] = range(len(df)) 

        fig, axs = plt.subplots(3, 2, figsize=(15, 10))
        fig.suptitle(f'Reporte de Rendimiento: {csv_filename}', fontsize=16)

        axs[0, 0].plot(df['segundos'], df['latency_ms'], color='tab:red', linewidth=2)
        axs[0, 0].set_title('Latencia por Inferencia')
        axs[0, 0].set_ylabel('Milisegundos (ms)')
        axs[0, 0].grid(True, linestyle='--', alpha=0.7)

        axs[0, 1].plot(df['segundos'], df['fps'], color='tab:green', linewidth=2)
        axs[0, 1].set_title('Fotogramas por Segundo (FPS)')
        axs[0, 1].set_ylabel('FPS')
        axs[0, 1].grid(True, linestyle='--', alpha=0.7)

        axs[1, 0].plot(df['segundos'], df['cpu_percent'], color='tab:blue', linewidth=2)
        axs[1, 0].set_title('Uso de CPU')
        axs[1, 0].set_ylabel('Porcentaje (%)')
        axs[1, 0].set_xlabel('Tiempo (muestras)')
        axs[1, 0].grid(True, linestyle='--', alpha=0.7)

        axs[1, 1].plot(df['segundos'], df['gpu_percent'], color='tab:purple', linewidth=2)
        axs[1, 1].set_title('Uso de GPU')
        axs[1, 1].set_ylabel('Porcentaje (%)')
        axs[1, 1].set_xlabel('Tiempo (muestras)')
        axs[1, 1].grid(True, linestyle='--', alpha=0.7)
        
        axs[2, 0].plot(df['segundos'], df['ram_mb'], color='tab:orange', linewidth=2)
        axs[2, 0].set_title('Consumo de Memoria RAM')
        axs[2, 0].set_ylabel('Megabytes (MB)')
        axs[2, 0].set_xlabel('Tiempo (muestras)')
        axs[2, 0].grid(True, linestyle='--', alpha=0.7)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        output_image = csv_filename.replace('.csv', '.png')
        plt.savefig(output_image)
        print(f"Gráfica guardada exitosamente como: {output_image}")
        
        plt.show()

    except FileNotFoundError:
        print(f"Error: No se encontró el archivo {csv_filename}")
    except Exception as e:
        print(f"Ocurrió un error: {e}")

if __name__ == "__main__":
    nombre_archivo = '/home/brad/ros2_ws/puzzlebot_ws/src/puzzlebot_vision/media/csv/cpu_metrics_yolo26N.onnx.csv' 
    generar_graficas_rendimiento(nombre_archivo)
