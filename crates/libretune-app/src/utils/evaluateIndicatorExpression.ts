// Modified 2026-11-01 in the HaniCore K-line fork of LibreTune (see FORK.md).
/**
 * Evaluates a simple boolean INI indicator expression against realtime +
 * constant values.
 *
 * Indicator expressions come from INI `bits`/indicator definitions and look
 * like `rpm > 2000 && map < 90` or just `launchControl`. This function turns
 * such an expression into a boolean so the UI can light up an indicator.
 *
 * # Evaluation
 *
 * `evalExpression` below parses the expression itself (C operator
 * precedence, numbers, identifiers, parentheses, `?:`). Nothing is ever
 * compiled: the release build's Content-Security-Policy forbids eval, and an
 * identifier can only ever become a number from `data`/`constants`.
 *
 * # Fast path
 *
 * An expression that is a single bare identifier (e.g. `checkEngine` or
 * `launchControl`) short-circuits: it is treated as truthy iff the resolved
 * value is defined and non-zero.
 *
 * # Fallback semantics
 *
 * - An unknown identifier resolves to `0` (so `missing > -1` is true, but
 *   `missing > 0` is false — matching TunerStudio's behaviour for undefined).
 * - ANY error during tokenize/build/eval returns `false` (fail-safe: an
 *   unparseable indicator expression lights nothing up rather than throwing).
 */
const TOKEN = /\s*(0x[0-9a-fA-F]+|\d+\.?\d*(?:[eE][-+]?\d+)?|\.\d+|[a-zA-Z_][a-zA-Z0-9_]*|&&|\|\||==|!=|<=|>=|<<|>>|[-+*/%()!~<>&|^?:])/y;

/** Parse and evaluate an INI expression with C operator precedence. Throws on
 * anything it does not understand (the caller turns that into `false`). */
export function evalExpression(src: string, vars: Record<string, number>): number {
  const toks: string[] = [];
  let i = 0;
  const text = src.replace(/[{}]/g, ' ');
  while (i < text.length) {
    if (/\s/.test(text[i])) { i++; continue; }
    TOKEN.lastIndex = i;
    const m = TOKEN.exec(text);
    if (!m) throw new Error('bad token at ' + i);
    toks.push(m[1]);
    i = TOKEN.lastIndex;
  }
  let p = 0;
  const peek = () => toks[p];
  const take = (t?: string) => {
    const x = toks[p++];
    if (t !== undefined && x !== t) throw new Error('expected ' + t);
    return x;
  };
  const LEVELS: string[][] = [['||'], ['&&'], ['|'], ['^'], ['&'], ['==', '!='],
    ['<', '<=', '>', '>='], ['<<', '>>'], ['+', '-'], ['*', '/', '%']];
  const apply = (op: string, a: number, b: number): number => {
    switch (op) {
      case '||': return a || b ? 1 : 0; case '&&': return a && b ? 1 : 0;
      case '|': return a | b; case '^': return a ^ b; case '&': return a & b;
      case '==': return a === b ? 1 : 0; case '!=': return a !== b ? 1 : 0;
      case '<': return a < b ? 1 : 0; case '<=': return a <= b ? 1 : 0;
      case '>': return a > b ? 1 : 0; case '>=': return a >= b ? 1 : 0;
      case '<<': return a << b; case '>>': return a >> b;
      case '+': return a + b; case '-': return a - b;
      case '*': return a * b; case '/': return a / b; case '%': return a % b;
    }
    throw new Error('op ' + op);
  };
  const unary = (): number => {
    const t = take();
    if (t === undefined) throw new Error('unexpected end');
    if (t === '!') return unary() ? 0 : 1;
    if (t === '-') return -unary();
    if (t === '+') return unary();
    if (t === '~') return ~unary();
    if (t === '(') { const v = ternary(); take(')'); return v; }
    if (/^(0x|\d|\.)/.test(t)) return Number(t);
    if (/^[a-zA-Z_]/.test(t)) return vars[t] ?? 0;
    throw new Error('unexpected ' + t);
  };
  const binary = (level: number): number => {
    if (level === LEVELS.length) return unary();
    let v = binary(level + 1);
    while (LEVELS[level].includes(peek())) {
      const op = take();
      v = apply(op, v, binary(level + 1));
    }
    return v;
  };
  const ternary = (): number => {
    const c = binary(0);
    if (peek() !== '?') return c;
    take('?');
    const a = ternary();
    take(':');
    const b = ternary();
    return c ? a : b;
  };
  const v = ternary();
  if (p !== toks.length) throw new Error('trailing ' + peek());
  return v;
}

export function evaluateIndicatorExpression(
  expression: string,
  data: Record<string, number>,
  constants: Record<string, number> = {},
): boolean {
  // Realtime data takes precedence over constants (spreads later override
  // earlier keys), so a name defined in both resolves to the live value.
  const context = { ...constants, ...data };

  try {
    const expr = expression.trim();

    // Fast path: a single bare identifier is truthy iff defined and non-zero.
    // Covers the common `indicatorName`-only expressions without compiling.
    if (/^[a-zA-Z_][a-zA-Z0-9_]*$/.test(expr)) {
      const val = context[expr];
      return val !== undefined && val !== 0;
    }

    // No eval: the release build's Content-Security-Policy (script-src
    // 'self', no 'unsafe-eval') makes `new Function` throw, so every
    // comparison here read false and indicator tiles never lit. The old
    // tokenizer also dropped + - * / | ? : silently. A small C-style parser
    // instead; an unknown identifier is 0 as before.
    return evalExpression(expr, context) !== 0;
  } catch {
    // Fail-safe: any error (bad expression, odd operator sequence, etc.)
    // yields false rather than propagating to the indicator render path.
    return false;
  }
}

/**
 * Collects every identifier referenced across one or more indicator
 * expressions. Used to know which realtime channels/constants must be
 * available before the expressions can be evaluated (e.g. to subscribe or to
 * surface a list of required channels). Purely lexical — does not evaluate.
 */
export function extractExpressionVariables(expressions: string[]): string[] {
  const vars = new Set<string>();
  for (const expression of expressions) {
    const tokens = expression.match(/[a-zA-Z_][a-zA-Z0-9_]*/g);
    tokens?.forEach((token) => vars.add(token));
  }
  return Array.from(vars);
}
