import js from '@eslint/js';
import globals from 'globals';
import tseslint from 'typescript-eslint';
import svelte from 'eslint-plugin-svelte';
import prettier from 'eslint-config-prettier';

export default tseslint.config(
  js.configs.recommended,
  ...tseslint.configs.recommended,
  ...tseslint.configs.stylistic,
  ...svelte.configs['flat/recommended'],
  prettier,
  {
    files: ['src/**/*.{ts,svelte}'],
    languageOptions: {
      globals: {
        ...globals.browser,
      },
    },
  }
);
