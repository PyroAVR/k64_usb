project('k64_base', 'c')

# dependencies
dependencies = []

# Project include path.
local_headers = ['include']

# List of C sources to compile. Relative to project root.
c_sources = [
    'src/status_leds.c',
    'src/hardfault.c',
    'src/systick.c',
    'src/main.c'
]

# List of ASM sources to compile.  Relative to project root.
asm_sources = [
]

## project setup
# CPU to use (default: on FRDM-K64F)
add_project_arguments('-DCPU_MK64FN1M0VLL12', language: 'c')

# use default clock?
# This is Kinetis-setup specific, and configures the Freescale MCG on boot.
add_project_arguments(['-DCLOCK_SETUP=1'], language: 'c')


### DON'T EDIT BELOW THIS LINE ###

# These are additional sources that are needed for normal system init, provided
# by NXP in the BSP.
c_sources = c_sources + ['core/src/system_MK64F12.c']
asm_sources = asm_sources + ['core/src/startup_MK64F12.S']

_includes = local_headers + ['core/include']
_incl_dirs = include_directories(_includes)

add_project_link_arguments(['-Wl,-Map=main.map'], language: 'c')
add_project_arguments(['-g3'], language: 'c')
add_project_arguments(['-D__STARTUP_CLEAR_BSS'], language: 'c')

asm_static_object = static_library(
    'asm_static_object',
    asm_sources,
    include_directories: _incl_dirs,
    c_args: get_option('c_args') + ['-x', 'assembler-with-cpp'],
    install: false
  )

main = executable(
    'main.elf',
    c_sources,
    include_directories: _incl_dirs,
    link_with: asm_static_object,
    dependencies: dependencies,
    install: true,
    install_dir: false
)
main_binary = custom_target(
    'main.bin',
    command: [meson.source_root() / 'scripts/create_bin.sh', 'main.elf', '@OUTPUT@'],
    depends: main,
    output: 'main.bin',
    build_by_default: true
)
