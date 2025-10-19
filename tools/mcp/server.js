#!/usr/bin/env node

const path = require('node:path');
const fs = require('node:fs/promises');
const { spawn } = require('node:child_process');
const { McpServer } = require('@modelcontextprotocol/sdk/server/mcp.js');
const { StdioServerTransport } = require('@modelcontextprotocol/sdk/server/stdio.js');
const { z } = require('zod');

const WORKSPACE_ROOT = path.resolve(__dirname, '..', '..');
const SCOREBOARD_PATH = path.join(WORKSPACE_ROOT, 'benchmarks3d', 'scoreboard.json');
const ARTIFACT_ROOT = path.join(WORKSPACE_ROOT, 'benchmarks3d', 'artifacts');
const WASM_SCRIPT = path.join(WORKSPACE_ROOT, 'build_wasm_quick.sh');

async function runCommand(cmd, args, options = {}) {
  return new Promise((resolve, reject) => {
    const child = spawn(cmd, args, { cwd: WORKSPACE_ROOT, shell: false, ...options });
    let stdout = '';
    let stderr = '';
    child.stdout?.on('data', (chunk) => {
      stdout += chunk.toString();
    });
    child.stderr?.on('data', (chunk) => {
      stderr += chunk.toString();
    });
    child.on('error', (error) => reject(error));
    child.on('close', (code) => {
      resolve({ code, stdout, stderr });
    });
  });
}

async function ensureScoreboard() {
  try {
    await fs.access(SCOREBOARD_PATH);
  } catch (_) {
    await fs.mkdir(path.dirname(SCOREBOARD_PATH), { recursive: true });
    await fs.writeFile(SCOREBOARD_PATH, '[]\n');
  }
}

async function readScoreboard() {
  await ensureScoreboard();
  const raw = await fs.readFile(SCOREBOARD_PATH, 'utf8');
  return JSON.parse(raw || '[]');
}

async function appendScoreboard(entry) {
  const entries = await readScoreboard();
  entries.push(entry);
  await fs.writeFile(SCOREBOARD_PATH, JSON.stringify(entries, null, 2) + '\n');
  return entries;
}

async function copyArtifacts(solver, timestampLabel, metrics) {
  const safeTimestamp = String(timestampLabel);
  const folder = `${safeTimestamp}-${solver}`;
  const targetDir = path.join(ARTIFACT_ROOT, folder);
  await fs.mkdir(targetDir, { recursive: true });

  const binaryName = process.platform === 'win32' ? 'solver-bench.exe' : 'solver-bench';
  const binarySource = path.join(WORKSPACE_ROOT, 'target', 'release', binaryName);
  const binaryTarget = path.join(targetDir, binaryName);
  await fs.copyFile(binarySource, binaryTarget);
  await fs.writeFile(path.join(targetDir, 'metrics.json'), JSON.stringify(metrics, null, 2) + '\n');

  return path.relative(WORKSPACE_ROOT, targetDir);
}

function parseCliArgs(argv) {
  const [command, ...rest] = argv.slice(2);
  return { command, args: rest };
}

async function handleBenchCli(args) {
  let solver = 'avbd';
  let steps = 120;

  for (let i = 0; i < args.length; i += 1) {
    const current = args[i];
    if (current === '--solver') {
      solver = args[i + 1] || solver;
      i += 1;
    } else if (current.startsWith('--solver=')) {
      solver = current.split('=')[1];
    } else if (current === '--steps') {
      steps = Number(args[i + 1] || steps);
      i += 1;
    } else if (current.startsWith('--steps=')) {
      steps = Number(current.split('=')[1]);
    }
  }

  const result = await runBench({ solver, steps });
  console.log(JSON.stringify(result.metrics, null, 2));
  console.log(`Artifacts copied to: ${result.artifactPath}`);
}

async function handleBuildCli(args) {
  let solver = 'avbd';
  let dim = '3';
  let release = true;

  for (let i = 0; i < args.length; i += 1) {
    const current = args[i];
    if (current === '--solver') {
      solver = args[i + 1] || solver;
      i += 1;
    } else if (current.startsWith('--solver=')) {
      solver = current.split('=')[1];
    } else if (current === '--dim') {
      dim = args[i + 1] || dim;
      i += 1;
    } else if (current.startsWith('--dim=')) {
      dim = current.split('=')[1];
    } else if (current === '--dev') {
      release = false;
    }
  }

  const output = await buildWasm({ solver, dim, release });
  if (output.stdout.trim()) {
    console.log(output.stdout.trim());
  }
  if (output.stderr.trim()) {
    console.error(output.stderr.trim());
  }
}

async function buildWasm({ solver, dim = '3', release = true }) {
  const args = [WASM_SCRIPT, dim, `--solver=${solver}`];
  if (!release) {
    args.push('--dev');
  }
  return runCommand('bash', args);
}

async function runBench({ solver, steps = 120 }) {
  const featureList = solver === 'avbd' ? 'solver_avbd,profiler' : 'solver_impulse,profiler';
  const args = [
    'run',
    '--package',
    'solver-bench',
    '--release',
    '--no-default-features',
    '--features',
    featureList,
    '--',
    '--solver',
    solver,
    '--steps',
    String(steps),
  ];

  const { code, stdout, stderr } = await runCommand('cargo', args);
  if (code !== 0) {
    throw new Error(`cargo run failed with code ${code}\n${stderr}`);
  }

  // Find the JSON output - collect all lines from first '{' to last '}'
  const lines = stdout.split(/\r?\n/).map(l => l.trim()).filter(l => l.length > 0);
  let jsonStartIdx = -1;
  let jsonEndIdx = -1;
  
  for (let i = 0; i < lines.length; i++) {
    if (lines[i].startsWith('{') && jsonStartIdx === -1) {
      jsonStartIdx = i;
    }
    if (lines[i].endsWith('}')) {
      jsonEndIdx = i;
    }
  }
  
  if (jsonStartIdx === -1 || jsonEndIdx === -1) {
    throw new Error(`unable to parse solver-bench output:\n${stdout}`);
  }
  
  const jsonText = lines.slice(jsonStartIdx, jsonEndIdx + 1).join('\n');
  const metrics = JSON.parse(jsonText);
  const commit = (await runCommand('git', ['rev-parse', 'HEAD'])).stdout.trim();
  await fs.mkdir(ARTIFACT_ROOT, { recursive: true });
  const artifactPath = await copyArtifacts(solver, metrics.timestamp_ms, metrics);
  const entry = { ...metrics, commit, artifactPath };
  await appendScoreboard(entry);

  return { metrics: entry, artifactPath };
}

function createMcpServer() {
  const server = new McpServer({
    name: 'rapier-avbd-mcp',
    version: '0.1.0',
  });

  server.registerTool(
    'build-wasm',
    {
      description: 'Builds the WASM package for the requested solver backend and dimension.',
      inputSchema: {
        solver: z.enum(['avbd', 'impulse']).default('avbd'),
        dim: z.enum(['2', '3']).default('3'),
        release: z.boolean().default(true),
      },
    },
    async ({ solver, dim, release }) => {
      const result = await buildWasm({ solver, dim, release });
      return {
        content: [
          {
            type: 'text',
            text: (result.stdout || '').trim() || 'build completed',
          },
        ],
      };
    }
  );

  server.registerTool(
    'run-bench',
    {
      description: 'Runs the solver-bench harness and records the results to the scoreboard.',
      inputSchema: {
        solver: z.enum(['avbd', 'impulse']).default('avbd'),
        steps: z.number().int().min(1).default(120),
      },
    },
    async ({ solver, steps }) => {
      const result = await runBench({ solver, steps });
      return {
        content: [
          {
            type: 'text',
            text: JSON.stringify(result.metrics, null, 2),
          },
        ],
        structuredContent: result.metrics,
      };
    }
  );

  server.registerTool(
    'get-scoreboard',
    {
      description: 'Returns the persisted benchmark scoreboard.',
      inputSchema: {},
    },
    async () => {
      const entries = await readScoreboard();
      return {
        content: [
          {
            type: 'text',
            text: JSON.stringify(entries, null, 2),
          },
        ],
        structuredContent: entries,
      };
    }
  );

  return server;
}

async function main() {
  const { command, args } = parseCliArgs(process.argv);

  try {
    if (!command || command === 'serve') {
      const server = createMcpServer();
      const transport = new StdioServerTransport();
      await server.connect(transport);
      console.error('MCP server running on stdio');
      return;
    }

    if (command === 'bench') {
      await handleBenchCli(args);
      return;
    }

    if (command === 'build') {
      await handleBuildCli(args);
      return;
    }

    console.error('Unknown command. Usage: node tools/mcp/server.js [serve|bench|build]');
    process.exit(1);
  } catch (error) {
    console.error(error instanceof Error ? error.message : String(error));
    process.exit(1);
  }
}

main();
