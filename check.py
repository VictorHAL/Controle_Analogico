#!/usr/bin/env python3
"""
Script de Verificação - Projeto Baratinha Controle Digital
Verifica todas as dependências e configurações necessárias
"""

import sys
import subprocess
import importlib.util

def check_color(passed):
    """Retorna cor ANSI para sucesso/falha"""
    return '\033[92m✓\033[0m' if passed else '\033[91m✗\033[0m'

def check_python_version():
    """Verifica versão do Python"""
    version = sys.version_info
    passed = version.major >= 3 and version.minor >= 8
    status = check_color(passed)
    print(f"{status} Python {version.major}.{version.minor}.{version.micro}", end="")
    if not passed:
        print(" (REQUER Python 3.8+)")
    else:
        print(" OK")
    return passed

def check_library(lib_name, import_name=None):
    """Verifica se uma biblioteca está instalada"""
    if import_name is None:
        import_name = lib_name
    
    spec = importlib.util.find_spec(import_name)
    passed = spec is not None
    status = check_color(passed)
    
    if passed:
        try:
            module = importlib.import_module(import_name)
            version = getattr(module, '__version__', 'unknown')
            print(f"{status} {lib_name:15s} v{version}")
        except:
            print(f"{status} {lib_name:15s} (instalado)")
    else:
        print(f"{status} {lib_name:15s} NÃO INSTALADO")
    
    return passed
def main():
    print("=" * 60)
    print("VERIFICAÇÃO DE AMBIENTE - Projeto Baratinha")
    print("=" * 60)
    
    results = {}
    
    # Python
    print("\n🐍 PYTHON:")
    results['python'] = check_python_version()
    
    # Bibliotecas Python
    print("\n📚 BIBLIOTECAS PYTHON:")
    libs = [
        ('numpy', 'numpy'),
        ('scipy', 'scipy'),
        ('matplotlib', 'matplotlib'),
        ('python-control', 'control'),
        ('sympy', 'sympy'),
        ('pandas', 'pandas'),
    ]
    
    results['libraries'] = all(check_library(name, imp) for name, imp in libs)
    
    # Resumo
    print("\n" + "=" * 60)
    print("RESUMO:")
    print("=" * 60)
    
    all_passed = all(results.values())
    
    if all_passed:
        print(f"{check_color(True)} Ambiente COMPLETO e pronto para uso!")
    else:
        print(f"{check_color(False)} Ambiente INCOMPLETO. Instale os itens faltantes:")
        print("\nPara instalar bibliotecas Python, execute:")
        print("  pip install -r requirements.txt")
        print("\nPara instalar PlatformIO:")
        print("  https://platformio.org/install")
    
    print("=" * 60)
    
    return 0 if all_passed else 1

if __name__ == '__main__':
    sys.exit(main())
