import pandas as pd
import numpy as np
from scipy import signal, stats
import matplotlib.pyplot as plt
from pathlib import Path
import seaborn as sns
from typing import Dict

class NoiseAnalyzer:
    
    timestamp_col = 'timestamp_us'
    value_col = 'raw_reading'

    def __init__(self, data_sources: Dict[str, str]):
        """
        Initialize analyzer with dictionary of data sources.
        Args:
            data_sources: Dict with format {'name': 'path_to_csv'}
        """
        self.data_sources = data_sources
        self.datasets = {}
        self.noise_stats = {}
        self._load_data()
        
    def _load_data(self):
        """Load all CSV files and perform initial preprocessing."""
        for name, path in self.data_sources.items():
            df = pd.read_csv(Path(path))
            # Assuming CSV has 'value' and 'timestamp' columns
            df[self.timestamp_col] = pd.to_datetime(df[self.timestamp_col], unit='us')
            # Calculate normalized timestamps in seconds starting from 0
            start_time = df[self.timestamp_col].min()
            df['time'] = (df[self.timestamp_col] - start_time).dt.total_seconds()

            # normalize timestamps to start from 0
            df[self.timestamp_col] = df[self.timestamp_col] - df[self.timestamp_col].min()
            df = df.set_index(self.timestamp_col)
            self.datasets[name] = df
            
    def calculate_noise_statistics(self):
        """Calculate various noise metrics for each dataset."""
        for name, df in self.datasets.items():
            # Detrend the signal to remove DC offset and linear trends
            detrended = signal.detrend(df[self.value_col])
            
            # Calculate noise metrics
            stats_dict = {
                'std': np.std(detrended),
                'peak_to_peak': np.ptp(detrended),
                'rms': np.sqrt(np.mean(detrended**2)),
                'kurtosis': stats.kurtosis(detrended),
                'skewness': stats.skew(detrended)
            }
            
            # Calculate power spectral density
            fs = 1 / (df.index[1] - df.index[0]).total_seconds()
            f, psd = signal.welch(detrended, fs, nperseg=1024)
            
            # Find dominant noise frequencies
            peak_freq_idx = np.argmax(psd)
            stats_dict['dominant_noise_freq'] = f[peak_freq_idx]
            stats_dict['peak_psd'] = psd[peak_freq_idx]
            
            # Store PSD for plotting
            stats_dict['frequencies'] = f
            stats_dict['psd'] = psd
            
            self.noise_stats[name] = stats_dict
            
    def plot_noise_analysis(self):
        """Generate comprehensive noise analysis plots."""
        # Create figure with subplots
        fig = plt.figure(figsize=(15, 10))
        gs = fig.add_gridspec(3, 2)
        
        # Time domain plot
        ax1 = fig.add_subplot(gs[0, :])
        for name, df in self.datasets.items():
            ax1.plot(df.index, signal.detrend(df[self.value_col]), label=name, alpha=0.7)
        ax1.set_title('Detrended Time Domain Signals')
        ax1.set_xlabel('Time')
        ax1.set_ylabel('Amplitude')
        ax1.legend()
        ax1.grid(True)
        
        # Power spectral density
        ax2 = fig.add_subplot(gs[1, :])
        for name, stats in self.noise_stats.items():
            ax2.semilogy(stats['frequencies'], stats['psd'], label=name, alpha=0.7)
        ax2.set_title('Power Spectral Density')
        ax2.set_xlabel('Frequency [Hz]')
        ax2.set_ylabel('PSD [V²/Hz]')
        ax2.legend()
        ax2.grid(True)
        
        # Histogram
        ax3 = fig.add_subplot(gs[2, 0])
        for name, df in self.datasets.items():
            sns.histplot(data=signal.detrend(df[self.value_col]), 
                        label=name, alpha=0.5, stat='density',
                        ax=ax3)
        ax3.set_title('Amplitude Distribution')
        ax3.set_xlabel('Amplitude')
        ax3.set_ylabel('Density')
        ax3.legend()
        
        # Stats table
        ax4 = fig.add_subplot(gs[2, 1])
        ax4.axis('off')
        
        # Create table data
        table_data = []
        metrics = ['std', 'peak_to_peak', 'rms', 'kurtosis', 'skewness', 'dominant_noise_freq']
        table_data.append(['Metric'] + list(self.noise_stats.keys()))
        
        for metric in metrics:
            row = [metric.replace('_', ' ').title()]
            for name in self.noise_stats.keys():
                value = self.noise_stats[name][metric]
                row.append(f'{value:.2e}' if abs(value) < 0.01 or abs(value) > 100 else f'{value:.3f}')
            table_data.append(row)
        
        table = ax4.table(cellText=table_data, loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1.2, 1.5)
        
        plt.tight_layout()
        return fig
    
    def generate_report(self):
        """Generate a text report of noise analysis."""
        report = []
        report.append("Noise Analysis Report")
        report.append("===================")
        
        for name in self.noise_stats.keys():
            report.append(f"\nResults for {name}:")
            report.append("-" * (len(name) + 13))
            stats = self.noise_stats[name]
            
            report.append(f"Standard Deviation: {stats['std']:.3e} V")
            report.append(f"Peak-to-Peak: {stats['peak_to_peak']:.3e} V")
            report.append(f"RMS Noise: {stats['rms']:.3e} V")
            report.append(f"Kurtosis: {stats['kurtosis']:.3f}")
            report.append(f"Skewness: {stats['skewness']:.3f}")
            report.append(f"Dominant Noise Frequency: {stats['dominant_noise_freq']:.2f} Hz")
            
            # Calculate noise bands
            f, psd = stats['frequencies'], stats['psd']
            bands = [
                (0, 1, "0-1 Hz"),
                (1, 10, "1-10 Hz"),
                (10, 100, "10-100 Hz"),
                (100, f[-1], f"100-{f[-1]:.0f} Hz")
            ]
            
            report.append("\nNoise Power in Frequency Bands:")
            for f_low, f_high, band_name in bands:
                mask = (f >= f_low) & (f < f_high)
                if any(mask):
                    power = np.trapz(psd[mask], f[mask])
                    report.append(f"  {band_name}: {power:.3e} V²")
        
        return "\n".join(report)

def main():
    # Example usage
    data_sources = {
        '660 SPS': Path(__file__).parent / 'sensor_data_20241123_154728.csv',
        '600 SPS': Path(__file__).parent / 'sensor_data_20241123_155813.csv',
        '600 SPS, PGA OFF': Path(__file__).parent / 'sensor_data_20241123_160222.csv',
        '600 SPS, PGA OFF, holding cable': Path(__file__).parent / 'sensor_data_20241123_161007.csv',
        
    }
    
    analyzer = NoiseAnalyzer(data_sources)
    analyzer.calculate_noise_statistics()
    
    # Generate and save plot
    fig = analyzer.plot_noise_analysis()
    fig.savefig('noise_analysis.png', dpi=300, bbox_inches='tight')
    
    # Generate and save report
    report = analyzer.generate_report()
    with open('noise_analysis_report.txt', 'w', encoding='utf-8') as f:
        f.write(report)

if __name__ == "__main__":
    main()